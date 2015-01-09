#ifndef EXCEPTION
#define EXCEPTION

#include "ATDA.hpp"

enum ErrorCode{
	EMPTY_METHOD_PARAM,
	EMPTY_VECTOR,
	NO_CONTOURS_FOUND,
	FILE_PROBLEM,
	BAD_INITIALIZE_PARAM,
	BAD_FUZZY_SET,
};

class ATDAException : std::exception{
	public:
		int err;
		string msg;
		string additionalMsg;
		string fullMsg;

	string getMsg(int err){
		switch(err){
		case EMPTY_METHOD_PARAM	: return "Error: Pusty parametr metody >  ";
		case EMPTY_VECTOR : return "Error: Pusty wektor > ";
		case NO_CONTOURS_FOUND : return "Error: Nie znaleziono konturu > ";
		case FILE_PROBLEM: return "Error: Problem z plikiem > ";
		case BAD_INITIALIZE_PARAM :  return "Error: Niewlasciwy parametr inicjalizacji > ";
		case BAD_FUZZY_SET :  return "Error: Niewlasciwy zbior rozmyty > ";
		}
	}

	const char *what() const throw() {
		const char *c = (this->fullMsg.c_str());
		return  c;
	}

	ATDAException(int errorName){
		this->err = errorName;
		this->msg = this->getMsg(this->err);
		this->fullMsg = this->msg;
	}

	ATDAException(int errorName, const string& additionalMsg){
		this->err = errorName;
		this->additionalMsg = additionalMsg;
		this->msg = this->getMsg(this->err);
		this->fullMsg = this->msg + this->additionalMsg;
	}
	
	~ATDAException() throw() {
	}
};

#endif
