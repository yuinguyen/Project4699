#ifndef CGUIDANCE_H
#define CGUIDANCE_H
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>




class CGuidance
{
    public:
    CGuidance();
    ~CGuidance();

    void update(cv::Mat imfromMain);
    //loop to get and store images and process them
    void get_im(cv::Mat &im);
    //threads for server images (didnt't end up using it though)
    void imthread();
    void imserver();

    bool find_target();
    //uses adjustWheels() and adjustServo() as input to control the car
    bool find_target_auto(std::string wheel_or_serv, int baseleftspeed, int baserightspeed);
    //function for wheel tracking the target to keep it at the center of the screen
    void adjustWheels(int centerPointX, int baseleftspeed, int baserightspeed);
    //didn't end up using this
    bool is_shot();
    //index initializer
    int i = 0;
    int j = 0;
    //flag checking if a target is found
    bool foundtarg1;
    bool foundtarg2;
    bool foundtarg3;
    bool foundtarg4;
    bool foundqrend;
    //flag to see whether a target is shot or not
    bool targ1shot;
    bool shoottarget1 = false;
    bool targ2shot;
    bool targ3shot;
    bool targ4shot;
    //flag telling that a state is done
    bool pathAdone;
    bool pathBdone;
    bool pathCdone;
    bool pathDdone;
    bool pathEdone;
    //pathD is special because you need to be on reverse
    bool turn_pathD = false;
    //flags for when looking for the target
    bool look_targ1;
    bool look_targ2;
    bool look_targ3;
    bool look_targ4;
    bool look_qrend;
    //flags for when to start looking for the next target
    bool start_look_targ2;
    bool start_look_targ3;
    bool start_look_targ4;
    bool start_look_qrend;

    //states
    void pathA(int init_pos);
    void pathB(int init_pos);
    void pathC(int init_pos);
    void pathD(int init_pos);
    void pathE(int init_pos);
    //counters
    int cnt ;
    int shootcnt;
    //get position
    int get_pos(int targID);
    bool at_center(int centerPointX);
    //detect marker
    cv::Mat detectMarker(cv::Mat inputImage);
    //auto shooter that tracks the marker with servo. Same concept to the adjustwheels
    void adjustServo(int centerPointX);
    //calculates the area, hence relative distance of the robot to a certain marker
    int get_area(cv::Point2f xy1, cv::Point2f xy2, cv::Point2f xy3, cv::Point2f xy4); //gets area as to calcualte relative distance
    //pulic variable for area
    int area;
    int set_area();
    //setting up a vector of markerson the screen
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    private:
    bool serverthreadexit;
    //servo position
    int servPos;
    //aruco dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
};

#endif // CGUIDANCE_H
