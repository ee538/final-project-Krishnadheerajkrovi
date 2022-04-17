#include "trojanmap.h"
#include<map>
#include <unordered_map>
#include<limits>
#include<bits/stdc++.h>
#include <algorithm>

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
  if(data.find(id)!=data.end())
  {
    return -1;
  }
  else
  {
    return data[id].lat;
    // return 0;
  }
    // return 0;
  //   return -1.0;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
  if(data.find(id)!=data.end())
  {
    return -1;
  }
  else
  {
    return data[id].lon;
    // return 0;
  }
    // return 0;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
    if(data.find(id)!=data.end())
  {
    return "";
  }
  else
  {
    return data[id].name;
  }
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    if(data.find(id)!=data.end())
  {
    return {};
  }
  else
  {
    return data[id].neighbors;
  }
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
for (auto str : data)
  {
    if (name == str.second.name)
    {
      return str.second.id;
    }
  }
  return "";
}
/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  for (auto str : data)
  {
    if (name == str.second.name)
    {
      std::pair<double, double> results(str.second.lat, str.second.lon);
      return results;
    }
  }
  std::pair<double, double> no_results(-1, -1);
  return no_results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){
  int m = a.size();
  int n = b.size();
  int D[2][m+1];
  for(int i=0;i<=m;i++)
    D[0][i] = i;
  for(int i=1;i<=n;i++)
  {
    for(int j=0;j<=m;j++)
    {
      if(j==0)
      D[i%2][j] = i; 
    else if(a[j-1] == b[i-1]){
      D[i%2][j] = D[(i-1)%2][j-1];
    }
    else{
      D[i%2][j]=1 + std::min(D[(i-1)%2][j],std::min(D[i%2][j-1],D[(i-1)%2][j-1]));
      }
    }
  }
  return D[n%2][m];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::map<std::string, int> memo;
  for (auto str:data)
  {
    
    std::string data_name = str.second.name;
    if (memo.count(data_name) < 1)
    {
      memo[data_name] = TrojanMap::CalculateEditDistance(name,data_name);
      // std::cout<<memo[data_name]<<std::endl;
    }
    
  }
  // return min_map.first;
  // return memo.begin()->first;
  std::pair<std::string, int> min_name ("Target",10000);
  for (auto element:memo)
  {
    if (element.second < min_name.second)
    {
      min_name.first = element.first;
      min_name.second = element.second;
    }
  }
  return min_name.first;
  // std::string tmp = "";
  // return tmp;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  name.erase(remove(name.begin(), name.end(), ' '), name.end());
  for(auto str : data){
    std::string loc_name = str.second.name;
    std::string lower="";
    lower.resize(loc_name.size());
    std::transform(loc_name.begin(),loc_name.end(),lower.begin(),::tolower);
    std::string lowername="";
    lowername.resize(name.size());
    std::transform(name.begin(),name.end(),lowername.begin(),::tolower);
    if(lower.find(lowername)==0){
      results.push_back(loc_name);
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
    std::vector<std::string> path;
    std::string start = GetID(location1_name);
    std::string end = GetID(location2_name);
    std::cout << start + "   " + end << std::endl; 
    std::priority_queue <std::pair<double,std::string>,std::vector<std::pair<double,std::string>>,std::greater<std::pair<double,std::string>>> minim_heap;                 //Priority queue to implement Heap with <distance,node ID>
    std::unordered_map <std::string,double> dist;                 //Unordered Map to store the distance from root to current node
    for(auto nodes:data){
      dist[nodes.second.id] = DBL_MAX;
    }
    std::unordered_map <std::string,std::string> predecessor;   //Unordered map to store the predecessor of the node
    std::unordered_map <std::string,bool> visited;          //Unord Map to keep track if node is visited
    for(auto nodes:data){
      visited[nodes.second.id] = false;
    }
    dist[start] = 0;
    minim_heap.push(std::make_pair(dist[start],start));
    while(!minim_heap.empty()){
      std::string current = minim_heap.top().second;
      minim_heap.pop();
      if(current!=end){
        //std::cout<<"HERE!!!";
        if(CalculateDistance(current,start)>dist[current]){
          continue;
        }
        else if(visited[current]){
          continue;
        }
        else{
          visited[current] = true;
          for(auto neighbour : data[current].neighbors){
            double new_dist = dist[current] + CalculateDistance(current,neighbour);
             if(dist[neighbour]>new_dist){
               dist[neighbour] = new_dist;
               predecessor[neighbour] = current;   
               minim_heap.push(std::make_pair(dist[neighbour],neighbour));          
          }
          }
        }
      }
      else{
      visited[end] = true;
      break;
      }
    }
    if(!visited[end])
     return path;
    for(auto node = end; node!= start; node = predecessor[node])
    {
      path.push_back(node);
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
  return path;
}
/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
    std::vector<std::string> path;
    std::string start = GetID(location1_name);
    std::string end = GetID(location2_name);
    std::unordered_map <std::string,double> dist;       //Unordered Map to store the distance from root to current node
    int f = 0;
    for(auto nodes:data){
      dist[nodes.second.id] = DBL_MAX;
    }
    std::unordered_map <std::string,std::string> predecessor; //Unordered map to store the predecessor of the node
    dist[start] = 0;
    //std::cout << data.size();
    if(start!=end){
      for (int i = 0; i < data.size()-1; i++){
        for (auto pair: data){
            std::string current = pair.second.id;
            for(auto neighbour : pair.second.neighbors){
                  double new_dist = dist[current] + CalculateDistance(current,neighbour);
                  if(dist[neighbour]>new_dist){
                    dist[neighbour] = new_dist;
                    predecessor[neighbour] = current;                    
                    f++;
                  }
            }
        }
        if(f==0){
              break;
            }
            f = 0;
      }
    }
    if(dist[end]==DBL_MAX)
      return path;
    for(auto node = end; node!= start; node = predecessor[node])
    {
      path.push_back(node);
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
    return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  return result;                                                     
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
