#ifndef PROPERTIES
#define PROPERTIES

namespace ROD {
	class Properties{
		// W³asnoœci stanowiska 
	private:
		int numViews;
		int numLines;
        static const int defaultViews = 10;
		static const int defaultLines = 5;
		// Struktura algorytmu
		int useSFS;
		int useSTEM;
            
	public:
        Properties(int v=defaultViews, int l=defaultLines, int sfs=0, int stem=0)
		{
			this->numViews = v; 
            this->numLines = l;
			this->useSFS = sfs;
			this->useSTEM = stem;

		}

		int getNumViews(){ return this->numViews; }
		int getNumLines(){ return this->numLines; }
		int getUseSFS(){ return this->useSFS; }
		int getUseSTEM(){ return this->useSTEM; }
	};
}

#endif

 