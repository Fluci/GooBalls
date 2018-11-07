#include<iostream>
#include<fstream>
#include<jsoncpp/json/json.h>

int main() {
	std::ifstream ifs("test.json");
	Json::Reader reader;
	Json::Value obj;
	reader.parse(ifs, obj);
	std::cout << "Last Name: " << obj["lastname"].asString() << std::endl;
	std::cout << "First Name: " << obj["firstname"] << std::endl;
	std::cout << "Legi: " << obj["legi"].asInt() - 1.5 << std::endl;
	// asInt() takes a number and rounds it down to the next int
	// asDouble() gives the number in double precision
	// without any .asXYZ() function it prints a Json::Value object, for a string this is "String"
	return 1;
}
