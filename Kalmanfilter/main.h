#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <stdio.h> //standard input output
#include <csignal>
#include <iostream>

void readDistance(Prog *p);
void server(Prog *p);
std::string handleZMQInput(Prog *p, std::string input);
std::string kalmanPos(std::string, marioKalman *m);
std::vector<double> split(std::string s, char c);	