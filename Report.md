# EE538 Final Project - TrojanMap
Authors: Nikhil Manjunath, Krishna Dheeraj Krovi

Video: https://youtu.be/hKBkNgVSnCw

## TrojanMap
Trojan Map maps the region around USC using C++ Data Structures and Graph Algorithms as shown below:

<p align="center"><img src="img/TrojanMap.png" alt="Trojan" width="500" /></p>

Each point on the map is represented by the class **Node** shown below and defined in [trojanmap.h](src/lib/trojanmap.h).

```cpp
class Node {
  public:
    Node(){};
    Node(const Node &n){id = n.id; lat = n.lat; lon = n.lon; name = n.name; neighbors = n.neighbors; attributes = n.attributes;};
    std::string id;    // A unique id assign to each point
    double lat;        // Latitude
    double lon;        // Longitude
    std::string name;  // Name of the location. E.g. "Bank of America".
    std::vector<std::string> neighbors;  // List of the ids of all neighbor points.
    std::unordered_set<std::string> attributes;  // List of the attributes of the location.
};
```

## Features

The following features have been implemented as part of the project:

```shell
TrojanMap
**************************************************************
* Select the function you want to execute.                    
* 1. Autocomplete                                             
* 2. Find the location                                        
* 3. CalculateShortestPath                                    
* 4. Travelling salesman problem                              
* 5. Cycle Detection                                          
* 6. Topological Sort                                         
* 7. Find Nearby                                              
* 8. Exit                                                     
**************************************************************
```
## Step 1: Autocomplete Feature

```c++
std::vector<std::string> Autocomplete(std::string name);
```

Autocomplete Function takes as input a partial location name from the user and returns the list of locations that start with the same characters as present in the input string. The function is Case Insensitive.

The time complexity of the Autocomplete function is **O(N)**.

<p align="center"><img src="img/Autocomplete.png" alt="Trojan" width="500" /></p>




