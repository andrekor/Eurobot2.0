/*
* File: prog.cpp
* Author: André Kramer Orten
*
* Contains the server from where the interaction between the robot and its 
* sensors is handled as well as the interaction with the positioning system.
*
* Copyright (c) 2015 André Kramer Orten <andrekor@ifi.uio.no>. All Rights Reserved.
*
* This file is part of EurobotUiO project.
*
* EurobotUiO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License  as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* EurobotUiO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See thread
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with EurobotUiO. If not, see <http://www.gnu.org/licenses/>.
*/
#include "log.h"
#include "prog.h"
#include "main.h"

std::string port = "5900"; //the port for zmq server
std::string comD = "/dev/ttyACM0"; //com for distance 
std::string comB = "/dev/ttyUSB0"; // com for beacon

/*Fetches the serial input. If its valid 
result it puts it sets the distance*/
void readDistance(Prog *p) {
	while(1) {
		std::string a = p->serialDistance->readLine();
		if (a.length() > 1) {
			std::string d1 = "0"; // temp
			std::string d2 = "0"; // temp
			std::string d3 = "0"; // temp
			std::string start = "0"; //temp
			int count;
			if (a.find("A") >= 0) {
				count = (a.find("B")-1)-a.find("A");
				d1 = a.substr(a.find("A")+1, count); // sone 1
			}
			if (a.find("B") > 0) {
				count = (a.find("C")-1)-a.find("B");
				d2 = a.substr(a.find("B")+1, count); // sone 2
			}
			if (a.find("C") > 0) {
				count = (a.find("D")-1)-a.find("C");
				d3 = a.substr(a.find("C")+1, count); // sone 3
			} if (a.find("D") > 0) {
				count = (a.length()-1)-a.find("D");
				start = a.substr(a.find("D"), count);
			}
 			//sets the distance variable in Prog
 			p->setStart(start);
 			p->setDistance(d1, d2, d3);
 		}
	}
}

/*
Splits the input string on c, and returns the 
values as double in a double vector
*/
std::vector<double> split(std::string input, char c) {
 	std::vector<double> args;
	std::istringstream f(input);
	std::string s;
	while(getline(f, s, ',')) {
		args.push_back(atof(s.c_str()));
	}
	return args;
}

/*Depending on the request, the server
should respond with either the position, 
or the distances from the distance sensors*/
void server(Prog *p) {
	//Prepare the context and socketudo netstat -taupen

	zmq::context_t context(1);
	zmq::socket_t socket(context, ZMQ_REP);
	std::stringstream stream;
	stream << "tcp://*:" << port;
	LOG("Starting server on " << stream.str());
	socket.bind(stream.str().c_str());
	while(true) {
		zmq::message_t request;
		//Wait for next request from client
		socket.recv(&request);
		//Fetch the request from client
		std::string rp1 = std::string(static_cast<char*>(request.data()), request.size());
		LOG("New request: " << rp1);	
		char c = rp1[0]; //The ID of the request
		std::string result = "-1"; //if no valid ID
		if (c == '1') {
			int count = (rp1.length()-1)-rp1.find(",");
			std::string stringPos = rp1.substr(rp1.find(",")+1, count);
			time_t now;
			time(&now); // same as now = time(NULL)
			if ((now - p->getTime())< 5) {
				result = kalmanPos(stringPos, p->mario);
				LOG("Reply with kalman position " << result);
			} else {
				/*Need to update the covariance matrix in the kalmanfilter*/
				result = stringPos;
				LOG("Reply with position " << result);
			}
		}
		else if (c == '2') {
			result = p->getDistance();
			LOG("Reply with distance " << result);
		} else if (c == '3') {
			result = p->getStart();
			LOG("Reply with start " << result);
		}
		zmq::message_t reply(result.length());
		memcpy ((void *) reply.data(), result.c_str(), result.length());
		socket.send(reply);
	}
}

std::string kalmanPos(std::string position, marioKalman *mario) {
	mat temp(3, 1); //temp matrice
	std::string s = position.substr(position.find(",")+1, position.length()); //contains y,theta
	double x = std::stod(position.substr(0, position.find(","))); 
    double y = std::stod(s.substr(0, s.find(",")));
    double theta = std::stod(s.substr(s.find(",")+1, s.length()));

	temp(0,0) = x;
	temp(1, 0) = y;
	temp(1, 0) = theta;
	
	mario->setState(temp); //Sets the state from encoders
	mario->predict(); //runs the predict phase
	mario->update(); // runs the update phase
	mat pos = mario->getState(); //Gets the state
	std::stringstream ss;
	ss << pos(0) << "," << pos(1) << "," << pos(2);
	return ss.str();
}

Prog::~Prog() {}

Prog::Prog() {
	serialDistance = new Serial(comD); //opens the communication to the distance arduino
	LOG("Opening serial Distance on" << comD);
	serialBeacon = new Serial(comB);  //opens the communication to the position arduino
	LOG("Opening serial Position on" << comB);
	distance1 = "0";
	distance2 = "0";
	distance3 = "0";
	prevMeasure = "0";
	start = "0"; 
	time(&timeSincePrevMeasure); //initialize to something high
	mario = new marioKalman(); //Initialize the kalman filter
	mario->setMeasure(22.0,100.0,0.0); //start position
}

void Prog::setStart(std::string s) {
	LOG("Should start " << s);
	start = s;
}

void Prog::setPrevMeasure(std::string measure) {
	time_t now;
	prevMeasure = measure;
	time(&now);  /* get current time; same as: now = time(NULL)  */
	std::vector<double> vec = split(measure, ',');
	if (vec.size() == 3) {
		mario->setMeasure(vec[0], vec[1], vec[2]);
		timeSincePrevMeasure = now;
		LOG("Sets the measure vector in kalman filter " <<  vec[0] << ", " << vec[1] << ", " << vec[2]);
	}
}

std::string Prog::getStart() {
	return start;
}

time_t Prog::getTime() {
	return timeSincePrevMeasure;
}

/*Sets the distance from the different sones*/
void Prog::setDistance(std::string dis1, std::string dis2, std::string dis3) {
	LOG(dis1 << "  -  " << dis2 << "  -  " << dis3);
	distance1 = dis1;
	distance2 = dis2;
	distance3 = dis3;
}

std::string Prog::getDistance() {
	std::stringstream ss;
	ss << distance1 << "," << distance2 << "," << distance3;
	return ss.str();
}

std::string Prog::getDistanceSone1() {
	return distance1;
}

std::string Prog::getDistanceSone2() {
	return distance2;
}

std::string Prog::getDistanceSone3() {
	return distance3;
}

/*Reads the input from the beacon tower, and saves the state. 
Perhaps this also is where the calculation should occure*/
void beaconPos(Prog *p) {
	while(1) {
		usleep(1000); 
		std::string a = p->serialBeacon->readLine(); //reads the line from the Position arduino
		if (a.length() > 1) {
			p->setPrevMeasure(a);
			//LOG("Beacon position " << a);
		}
	}
}

/*Checks the arguments given to the program*/
void handleInput(int argc, char *argv[]) {
	std::stringstream ss;
	std::stringstream cs;
	std::string p = "-p";
	std::string d = "-d";
	std::string b = "-b";
	for (int i = 1; i < argc; i+= 2) {
		if (argc > i+1) {
			cs << argv[i];	
			if (p.compare(cs.str())== 0) {
				ss << argv[i+1];
				port = ss.str(); //setting the port
				//LOG("setting port to " << port);
			} else if(d.compare(cs.str()) == 0) {
				ss << argv[i+1];
				comD = ss.str();
				//LOG("setting the serial distance port to " << comD);
			} else if(b.compare(cs.str())==0) {
				ss << argv[i+1];
				comB = ss.str();
				//LOG("setting the serial beacon port to " << comB);
			}
			// clear the string streams
			cs.str(std::string()); 
			ss.str(std::string());
		}
	}
}

int main(int argc, char *argv[]) {
	//std::string port = "5900"; //test variable
	if (argc > 2) {
		handleInput(argc, argv);
	}
	Prog *p = new Prog();
	//Threads the distance reader, the zmq server and beacon position
	LOG("Starts the distance thread");
	std::thread distance(readDistance, p);
	LOG("Starts the server thread");
	std::thread zmq(server, p);
	LOG("Starts beacon position thread");
	std::thread beacon(beaconPos, p);

	//Joins the threads with the main thread(kills the thread)
	if (distance.joinable())
		distance.join();
	if (zmq.joinable())
		zmq.join();
	if (beacon.joinable()) 
		beacon.join();
} 
