# robot_localization
## By Andrew Pan and Robbie Siegel

### Writeup
Our implementation of the particle filter aimed to determine the location of a robot on a given map utilizing sensor measurements from the robot’s Lidar and internal odometry calculations.  Our approach to implementing the particle filter involved breaking down the holistic functionality into a few key features, determining the inputs and outputs of each feature, and developing said features individually then later connecting everything together.  More specifically, our overall particle filter was comprised of reading in odometry data to calculate the robot’s change in position over a given time interval, generation of multiple potential locations the robot could be at (particles) based on previously stored potential locations with noise added to movement, evaluating the confidence of each calculated particle based on Lidar data, and updating/deciding which particles to keep for the next iteration.  A couple of design decisions we made involved simplifying the calculations we performed for the sake of simplicity so we could produce a working product faster, such as simply averaging new confidence measurements with the previous confidence measurement rather than performing a more robust Bayesian calculation, which we did because a simple average still accounted for previous results to an extent.  Some challenges we faced primarily involved debugging and accounting for differences in the units presented by the map, mainly ensuring that we multiplied values taken from the map by the map’s resolution.  Given more time, we would have used a more robust implementation of confidence evaluations, as we chose to simply use the provided function from OccupancyField, though ideally we would have preferred to attempt to match the shape of the surroundings provided through LaserScan data to shapes on the map of the space the robot moved around in, and calculate some confidence for locations based on the accuracy of the match.  We learned from this project that after the development of each individual component of a project, the component should be appropriately tested with both unit and integration tests to ensure that it functions as expected and will mesh with other components when it comes time to actually integrate our individually written code together.
