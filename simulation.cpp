#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

struct Vector3D {
    double x,y,z;
    Vector3D(double x_ = 0, double y_ = 0, double z_ = 0):x(x_),y(y_), z(z_) {}
    double magnitude() const { return std::sqrt(x*x + y*y + z*z); }
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x+other.x, y+other.y, z+other.z);
    }
};

class Rocket {
private:
    // Customizable parameters
    double mass; // Kg
    double fuelMass; // Kg
    double thrust; // Newtons
    double fuelBurnRate; // kg/s
    double launchAngle; // radians
    Vector3D windVelocity; // m/s
    double gravity; // m/s^2
    double dragCoeff; // dimensionless
    double airDensity; // kg/m^3
    double crossSection; // m^2

    // State variables
    Vector3D position;
    Vector3D velocity;
    Vector3D acceleration;
    std::vector<Vector3D> trajectory;
    std::vector<std::pair<double, std::string>> events;
    bool hasLanded = false;

public:
    Rocket(const std::string& configFile) {
        loadConfig(configFile);
        position = Vector3D(0,0,0);
        velocity = Vector3D(0,0,0);
        acceleration = Vector3D(0,0,0);
        trajectory.push_back(position);
        events.push_back({0.0, "Launch"});
    }

    void loadConfig(const std::string& filename) {
        std::ifstream file(filename);
        if(!file.is_open()) {
            std::cerr << "Error opening config file: " << filename << "\n";
            // Set default values
            mass = 1000.0; fuelMass = 500.0; thrust = 20000.0; fuelBurnRate = 10.0;
            launchAngle = 45.0 * M_PI / 180.0; windVelocity = Vector3D(5.0, 0.0, 0.0);
            gravity = 9.81; dragCoeff = 0.75; airDensity = 1.225; crossSection = 1.0;
            return;
        }

        std::string line;
        while(std::getline(file, line)) {
            std::istringstream iss(line);
            std::string key;
            double value1, value2, value3;
            if (std::getline(iss, key, '=') && key.find('#') == std::string::npos) {
                if(key == "windVelocity") {
                    if (iss >> value1 >> value2 >> value3) {
                        windVelocity = Vector3D(value1, value2, value3);
                    }
                }
                else{
                    if (iss >> value1) {
                        if (key == "mass") mass = value1;
                        else if (key == "fuelMass") fuelMass = value1;
                        else if (key == "thrust") thrust = value1;
                        else if (key == "fuelBurnRate") fuelBurnRate = value1;
                        else if (key == "launchAngleDeg") launchAngle = value1 * M_PI / 180.0;
                        else if (key == "gravity") gravity=value1;
                        else if (key == "dragCoeff") dragCoeff = value1;
                        else if (key == "airDensity") airDensity = value1;
                        else if (key == "crossSection") crossSection = value1;
                    }
                }
            }
        }
        file.close();
        std::cout << "Configuration loaded from " << filename << "\n";
    }

    void update(double deltaTime, double currentTime) {
        bool hasFuelRemaining = fuelMass > 0;
        if (fuelMass > 0) {
            fuelMass -= fuelBurnRate * deltaTime;
            if (fuelMass <= 0) {
                fuelMass = 0;
                events.push_back({currentTime, "Fuel Depleted"});
            }
        }

        double currentMass = mass + fuelMass;
        Vector3D thrustDir(std::sin(launchAngle), std::cos(launchAngle), 0);
        double thrustMagnitude = (fuelMass > 0)? thrust:0;
        Vector3D thrustForce(thrustDir.x * thrustMagnitude, thrustDir.y * thrustMagnitude, thrustDir.z * thrustMagnitude);
        Vector3D gravityForce(0, -currentMass * gravity, 0);
        Vector3D relativeVelocity = velocity + Vector3D(-windVelocity.x, -windVelocity.y, -windVelocity.z);

        double speed = relativeVelocity.magnitude();
        double dragMagnitude = 0.5 * airDensity * speed * speed * dragCoeff * crossSection;
        Vector3D dragForce;
        if (speed > 0) {
            dragForce = Vector3D(
                -relativeVelocity.x * dragMagnitude / speed,
                -relativeVelocity.y * dragMagnitude / speed,
                -relativeVelocity.z * dragMagnitude / speed
            );
        }

        Vector3D netForce = thrustForce + gravityForce + dragForce;
        acceleration = Vector3D(netForce.x / currentMass, netForce.y / currentMass, netForce.z / currentMass);
        velocity.x += acceleration.x * deltaTime;
        velocity.y += acceleration.y * deltaTime;
        velocity.z += acceleration.z * deltaTime;

        if (position.y <= 0 && !hasLanded) {
            position.y = 0;
            velocity.y = 0;
            if (currentTime > 0) {
                events.push_back({currentTime, "Landing"});
                hasLanded = true;
            }
        }

        trajectory.push_back(position);
    }

    void printStatus() {
        std::cout << "Position (x,y,z): (" << position.x << ", " << position.y << ", " << position.z << ") m\n";
        std::cout << "Velocity (x,y,z): (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ") m/s\n";
        std::cout << "Acceleration (x,y,z): (" << acceleration.x << ", " << acceleration.y << ", " << acceleration.z << ") m/s^2\n";
        std::cout << "Total Mass: " << (mass + fuelMass) << " kg (Fuel: " << fuelMass << " kg)\n";
        std::cout << "Wind (x,y,z): (" << windVelocity.x << ", " << windVelocity.y << ", " << windVelocity.z << ") m/s\n";
        std::cout << "----------------------------------------\n";
    }

    void exportToFile(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << filename << "\n";
            return;
        }

        file << "time,x,y,z,event\n";
        double timeStep = 0.1;
        for (size_t i = 0; i < trajectory.size(); ++i) {
            double time = i * timeStep;
            std::string event = "";
            for (const auto& e : events) {
                if (std::abs(e.first - time) < timeStep / 2) {
                    event = e.second;
                    break;
                }
            }
            file << time << "," << trajectory[i].x << ", " << trajectory[i].y << ", " << trajectory[i].z << ", " << "\"" << event << "\"\n";
        }
        file.close();
        std::cout << "Trajectory data export to " << filename << "\n";
    }
};

int main() {
    Rocket myRocket("rocket_config.txt");

    double timeStep = 0.1;
    double totalTime = 20.0;

    std::cout << "3D Rocket Simulation Starting...\n";

    for (double t = 0; t <= totalTime; t += timeStep) {
        std::cout << "Time: " << t << "s\n";
        myRocket.update(timeStep, t);
        myRocket.printStatus();
    }

    myRocket.exportToFile("rocket_trajectory.csv");
    return 0;
}