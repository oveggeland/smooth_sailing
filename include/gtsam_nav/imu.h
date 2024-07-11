#include "graph.h"
#include "sensor_msgs/Imu.h"

using namespace std;

#define IMU_DEBUG (false)

class IMUHandle{
    public:
        // Constructor
        IMUHandle(GraphHandle* p_gh);

        // Commen entry point for new imu messages
        void newMsg(sensor_msgs::Imu::ConstPtr msg);

    private:
        GraphHandle* p_gh;
        Unit3 gravity;

        void integrateMeasurement(sensor_msgs::Imu::ConstPtr msg);
        void initializeOrientation(sensor_msgs::Imu::ConstPtr msg);
};