#include "NodeClass.h"
#include <aod/AnalysisConfig.h>
#include <sorter_msgs/streamIN.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

// dla ROD
#include "ROD/Parameters.h"
#include "ROD/Properties.h"
#include "ROD/UserParameters.h"
#include "ROD/ResultOD.h"
#include "ROD/Analysis.hpp"

// dla SFS 
#include "SFS/Resolver.hpp"
#include "SFS/ATDA.hpp"
#include "SFS/Calibrator.hpp"

class Analysis {
public:
	Analysis(){
		sub_=nh_.subscribe("/rod/analysis/streamIN", 1000, &Analysis::messageCallbackStreamIN, this);
		pub_message_ = nh_.advertise<sorter_msgs::streamOUT>("/rod/analysis/streamOUT", 1000);
		
		// Praca z programem:
		// Należy utworzyć i zainicjalizować obiekty klasy Analysis i Calibrator
		// a następnie wywołać metody Analysis.OcenaDefektow() - dla każdego widoku jabłka - i
		// Analysis.DefektyJablka(): dla integracji wyników dla kilku widoków jabłek w liniach
		
		// Przełączniki pomocnicze: dla wizualizacji, monitorowania i "utrwalania" wyników 
		const int VIS = 0;  // wizualizacja obrazów
		const int MONIT = 0; // debug - tekst
		const int LOG = 0; // utrwalenie wyników

		// KROK 0:
		// INICJALIZACJA obiektów

		// Obiekt z własnościami stanowiska
		ROD::Properties pr = ROD::Properties(8, 1, 0); // liczba widoków i linii (różnych jabłek); domyślnie =(8, 3), useSFS, useSTEM
		// Obiekt z parametrami analizy obrazu
		ROD::Parameters pa = ROD::Parameters(16, 0.2, 0.5, 0.20, 50, 20, 720); // low edge, edge threshold, corner max, corner min, red color thresh min, intensity thresh min, direction's number
		// Obiekt z parametrami definiowanymi przez użytkownika
		ROD::UserParameters	upa = ROD::UserParameters(1); // typ jabłka: 1 - czerwony
		
		// Obiekt z parametrami biblioteki SFS 3D (ATDA)
		SFSParameters sfspa= SFSParameters(cv::Point3d(0.001, 0.001, 1.0), 1, 11, 11.0, 10, 20, 8);
		
		// Utwórz obiekt klasy Analysis zdefiniowanej w przestrzeni nazw ROD
		rodobj = new ROD::Analysis(&sIN, pr, pa, upa, &sOUT, VIS, MONIT, LOG);

		// Inicjalizuj obiekt dla analizy 3D metodą sfs
		rodobj->sfs3D = new Calibrator(sfspa, "/home/mstefanc/ws_sorter/vision/src/aod/src/SFS/mlp.xml", VIS, MONIT, LOG);  // obiekt główny biblioteki SFS
		
		// Alokuj pamięć wyników cząstkowych
		storedResults = new ROD::ResultOD[pr.getNumViews()];
		
		results = new ROD::ResultOD[pr.getNumLines()];
	}
	
	~Analysis() {
		delete rodobj->sfs3D;
		delete rodobj;
	}
	
	void callback(aod::AnalysisConfig &config, uint32_t level)
	{ 
	 
	  ROS_INFO("Reconfigure request, parameter: %f", config.parameter);
	}
	
	bool checkIn(int x, int y, int cx, int cy, int r) {
		float dx_l = x-(cx-r);
		float dy_l = y-(cy);
		
		float dx_r = x-(cx+r);
		float dy_r = y-(cy);
		
		float rr = 1.5 * r;
		
		return ((dx_l*dx_l + dy_l*dy_l < rr*rr) && (dx_r*dx_r + dy_r*dy_r < rr*rr));
	}
	
	void messageCallbackStreamIN( const sorter_msgs::streamIN::ConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		
		int sum[8];
		for (int i = 0; i < 8; ++i) sum[i] = 0;
		
		ROS_ERROR("streamIN: %d %d", (int)msg->ir.size(), (int)msg->rgb.size());
		for (int i = 0 ; i < (int)msg->ir.size(); ++i) {
			
			cv_bridge::CvImagePtr cv_ptr;
			
			ROS_ERROR("Analiza %d/%d", i, (int)msg->ir.size());
			
			cv_ptr = cv_bridge::toCvCopy(msg->ir[i], "mono8");
			cv::Mat ir_img = cv_ptr->image;
			cv_ptr = cv_bridge::toCvCopy(msg->rgb[i], "rgb8");
			cv::Mat rgb_img = cv_ptr->image;
			
			// Blok "główny"
			// 1) Wywołanie funkcji detekcji i oceny defektów (metoda w klasie ROD::Analysis)
			storedResults[i] = rodobj->AnalizaJablka(NULL, NULL, "/tmp", "out", i, ir_img, rgb_img); // wynik zwracany jest w storedResults 
			
			cv::Mat res = storedResults[i].regions;
			
			cv::Mat regs = rgb_img.clone();
			cv::cvtColor(regs, regs, CV_RGB2BGR);
			ROS_ERROR("Regions: %dx%d, regnum: %d", res.rows, res.cols, storedResults[i].numReg);
			int cx = storedResults[i].cx;
			int cy = storedResults[i].cy;
			int r = storedResults[i].radius;
			
			for (int r = 0; r < res.rows; ++r) {
				if ((storedResults[i].classReg[r] < 1) || (storedResults[i].classReg[r] > 7)) continue;
			//	ROS_ERROR("%d: %lf %lf %lf %lf %lf %lf", storedResults[i].classReg[r], res.at<double>(r, 0), res.at<double>(r, 1), res.at<double>(r, 2), res.at<double>(r, 3), res.at<double>(r, 4), res.at<double>(r, 5));
				int xx = res.at<double>(r, 4);
				int yy = res.at<double>(r, 3);
				if (checkIn(xx, yy, cx, cy, r)) {
					int rad = sqrt(res.at<double>(r, 2) / 3.14);
					cv::Scalar color = cv::Scalar(0, 0, 0);
					if (storedResults[i].classReg[r] == 1) color = cv::Scalar(255, 255, 255);
					if (storedResults[i].classReg[r] == 2) color = cv::Scalar(255, 0, 0);
					if (storedResults[i].classReg[r] == 3) color = cv::Scalar(255, 255, 0);
					if (storedResults[i].classReg[r] == 4) color = cv::Scalar(0, 255, 255);
					if (storedResults[i].classReg[r] == 5) color = cv::Scalar(0, 0, 255);
					if (storedResults[i].classReg[r] == 6) color = cv::Scalar(0, 255, 0);
					if (storedResults[i].classReg[r] == 7) color = cv::Scalar(255, 0, 255);
					cv::circle(regs, cv::Point(res.at<double>(r, 4), res.at<double>(r, 3)), rad, color);
					//sum[storedResults[i].classReg[r]]
				}
			}
			
			
			cv::circle(regs, cv::Point(cx, cy), r, cv::Scalar(0, 255, 0));
			cv::circle(regs, cv::Point(cx+r, cy), 1.5*r, cv::Scalar(0, 255, 0));
			cv::circle(regs, cv::Point(cx-r, cy), 1.5*r, cv::Scalar(0, 255, 0));
			
			
			cv::imshow("out_1", storedResults[i].classImage);
			cv::imshow("out_2", regs);
			cv::waitKey(10);
		}
        
		// 2) Integruj wyniki widoków jednego jabłka        
		//rodobj->DefektyJednegoJablka(storedResults, results); // Funkcję implementuje M.Stefańczyk
	}
	
private:
	ros::Publisher pub_message_;
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	
	// obiekty potrzebne do analizy obrazów
	ROD::Analysis *rodobj; // obiekt główny
	ROD::ResultOD *storedResults; // pomocnicza struktura wynikow częściowych
	ROD::ResultOD *results;
	ROD::ResultOD sOUT;
	cv::Mat  sIN;
};




int main(int argc, char **argv)
{

  ros::init(argc, argv, "analysis");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<aod::AnalysisConfig> srv;
  dynamic_reconfigure::Server<aod::AnalysisConfig>::CallbackType f;
  
  Analysis analysis_;
  
  f = boost::bind(&Analysis::callback, &analysis_, _1, _2);
  srv.setCallback(f);
  
  ros::Rate loop_rate(10);
  ROS_INFO("Starting to spin...");
  ros::spin();

  return 0;
}

