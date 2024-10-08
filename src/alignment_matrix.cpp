
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"

#include "smooth_sailing/alignmentFactor.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Point2.h"
#include <fstream>
#include <filesystem>

#include "ros/ros.h"

namespace fs = std::filesystem;
using namespace gtsam;
using namespace std;


#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

vector<Rot3> readShipOrientation(std::string filename){
    vector<Rot3> rotations; // Vector to hold the Rot3 objects
    ifstream file(filename); // Open the file
    string line;

    // Check if the file is opened successfully
    if (!file.is_open()) {
        cerr << "Could not open the file: " << filename << endl;
        return rotations; // Return empty vector on failure
    }

    getline(file, line); // Read the header

    // Read the file line by line
    while (getline(file, line)) {
        stringstream ss(line);
        string token;

        // Read the first token (timestamp), ignore it
        getline(ss, token, ','); // ts

        // Read the next three tokens for x, y, z (if needed, ignore them)
        getline(ss, token, ','); // x
        getline(ss, token, ','); // y
        getline(ss, token, ','); // z

        // Read the next three tokens for roll, pitch, heading
        double roll, pitch, heading;
        getline(ss, token, ','); // roll
        roll = std::stod(token) * DEG2RAD;
        getline(ss, token, ','); // pitch
        pitch = std::stod(token) * DEG2RAD;
        getline(ss, token, ','); // heading
        heading = std::stod(token) * DEG2RAD;

        // Create a Rot3 object from the Euler angles (roll, pitch, heading)
        Rot3 rotation = Rot3::RzRyRx(roll, pitch, heading);
        rotations.push_back(rotation); // Store the Rot3 object in the vector
    }

    file.close(); // Close the file
    return rotations; // Return the vector of Rot3 objects
}


// Function to read CSV file and return vector of Point2 objects (containing roll and pitch)
vector<Point2> readRollPitchFromCSV(const string& filename) {
    vector<Point2> rollPitch; // Vector to hold the Point2 objects for roll and pitch
    ifstream file(filename); // Open the file
    string line;

    // Check if the file is opened successfully
    if (!file.is_open()) {
        cerr << "Could not open the file: " << filename << endl;
        return rollPitch; // Return empty vector on failure
    }

    // Read and skip the header line
    getline(file, line); // Read the header

    // Read the file line by line
    while (getline(file, line)) {
        stringstream ss(line);
        string token;

        // Read the first seven tokens (timestamp, x, y, z, vx, vy, vz), ignore them
        for (int i = 0; i < 7; ++i) {
            getline(ss, token, ','); // Skip the first seven tokens
        }

        // Read the next two tokens for roll and pitch
        double roll, pitch;
        getline(ss, token, ','); // roll
        roll = std::stod(token);
        getline(ss, token, ','); // pitch
        pitch = std::stod(token);

        // Create a Point2 object from roll and pitch and add it to the vector
        Point2 rp(roll, pitch);
        rollPitch.push_back(rp); // Store the Point2 object in the vector
    }

    file.close(); // Close the file
    return rollPitch; // Return the vector of Point2 objects
}


int main(int argc, char** argv){
    ros::init(argc, argv, "alignment_node");
    ros::NodeHandle nh("~");
    
    // Find workspace
    std::string workspace, exp;
    if (!nh.getParam("/ws", workspace) || !nh.getParam("/exp", exp)){
        cout << "Error: Workspace or experiment namespace not provided" << endl;
        exit(1);
    }

    string ship_data = fs::path(workspace) / "exp" / exp / "navigation" / "ship_aligned.csv";
    fs::path nav_data = fs::path(workspace) / "exp" / exp / "navigation" / "nav_aligned.csv";

    vector<Rot3> ship_rotation = readShipOrientation(ship_data);
    vector<Point2> nav_attitude = readRollPitchFromCSV(nav_data);


    // Initialize a factor graph
    auto noise = noiseModel::Isotropic::Sigma(2, 0.01);
    int alignmentKey = 0;

    NonlinearFactorGraph graph = NonlinearFactorGraph();
    Values initial_values;
    initial_values.insert(alignmentKey, Rot3::RzRyRx(0, -0.5, -0.9*M_PI/2));

    // graph.addPrior(key, Rot3::RzRyRx(0, 0, M_PI/2));

    for (int i = 0; i < ship_rotation.size(); i++){
        auto factor = AlignmentFactor(alignmentKey, nav_attitude[i], ship_rotation[i], noise);
        graph.add(factor);
    }

    LevenbergMarquardtParams p;
    p.setVerbosityLM("SUMMARY");

    auto optimizer = LevenbergMarquardtOptimizer(graph, initial_values, p);
    Values optimal_values = optimizer.optimize();
    
    Rot3 R_align = optimal_values.at<Rot3>(alignmentKey);
    cout << "Alignemnt matrix is " << R_align << endl;
    cout << "RPY is: " << R_align.rpy()*RAD2DEG << endl;

    return 0;
}