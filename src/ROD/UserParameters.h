#ifndef USER_PARAMETERS
#define USER_PARAMETERS

namespace ROD {
	
	const int defaultType = 1; //  redApple = 1; greenApple = 2; yellowApple = 3;

	class UserParameters{
	private:
		int type; // typ jab�ka
        
	public:
		UserParameters(int tt=defaultType)
		{
			this->type = tt; // typ jab�ka
		}
	};
}
#endif
                