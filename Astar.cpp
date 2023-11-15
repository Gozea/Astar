#include <cmath>
#include <iostream>
#include <chrono>
#include <SFML/Graphics.hpp>
#include <eigen3/Eigen/Dense>
#include <iterator>
#include <queue>

using namespace std;

const int WIDTH = 100;
const int HEIGHT = 100;

struct Edge;
struct Node;

struct Edge {
    std::pair<Node*, Node*> nodes;
	float cost;

    // Constructor for Edge
    Edge(Node* node1, Node* node2, float c) : nodes(std::make_pair(node1, node2)), cost(c) {}
};

struct Node {
    int x;
    int y;
    std::vector<Edge> edges;

    // redefine equality operator
    bool operator==(Node n) { return x == n.x && y == n.y ;}
};
Node nodes[WIDTH*HEIGHT];




void createGrid() {
	//add edges to nodes
	for (int i = 0 ; i < HEIGHT ; i++) {
		for (int j = 0 ; j < WIDTH ; j++) {
		        // set node coordinates
		        nodes[j + i*WIDTH].x = j;
		        nodes[j + i*WIDTH].y = i;
			//exploring neighborhood
			for (int m = -1 ; m <= 1 ; m++) {
				for (int n = -1 ; n <= 1 ; n++) {
					//not adding edge to self or out of bound
					if(!(m == 0 && n == 0) && ((i+m) >= 0 && (i+m) < HEIGHT) && (j+n) >=0 && (j+n) < WIDTH) {
						//check whether neighbor is diagonal or not
						float c = (m == 0 || n == 0) ? 1 : std::sqrt(2);
                        // create the edge and add it
                        Edge e = Edge(&nodes[j + i*WIDTH], &nodes[j+n + (i+m)*WIDTH], c);
                        nodes[j + i*WIDTH].edges.push_back(e);
					}
				}
			}
		}
	}
}

std::unordered_map<Node*, float> initCost(Node* startNode) {
    std::unordered_map<Node*, float> costs;
    costs[startNode] = 0;
    for(Node& node : nodes) {
        if(!(node == *startNode)) {
            costs[&node] = std::numeric_limits<float>::max();
        }
    }
    return costs;
}

// return node from cooordinates
Node* getNode(int x, int y) {
    return &nodes[x + y*WIDTH];
}

// override of the getNode if we get Vector2i as parameter
Node* getNode(sf::Vector2i coord) {
    return &nodes[coord.x + coord.y*WIDTH];
}

// the node is not actually removed from the graph but it gets isolated from its neighbors
void removeNode(int x, int y) {
    Node* node = getNode(x, y);
    // for each node we go through its edges to get the corresponding neighbors
    std::for_each(node->edges.begin(), node->edges.end(), [&node](Edge& edge) {
        Node* neighbor_node = edge.nodes.second;
        // go through the edges of the neighbor to find the original node and erase the edge with it
        neighbor_node->edges.erase(std::remove_if(neighbor_node->edges.begin(), 
                                   neighbor_node->edges.end(), 
                                   [&node](Edge& neighbor_edge) { return *neighbor_edge.nodes.second == *node; 
        }));
    });
    // remove all edges of the original node
    node->edges.clear();
}


// Return number of connected nodes (isolated nodes are not counted)
int getSize() {
    int size = 0;
    for (int i=0; i < HEIGHT ; i++) {
        for (int j=0; j < WIDTH ; j++) {
            if (nodes[j + i*WIDTH].edges.size() != 0) size++;
        }
    }
    return size;
}

// Return vector of isolated nodes
vector<Node*> getObstacles() {
    vector<Node*> obstacles;
    for (int i=0; i < HEIGHT ; i++) {
        for (int j=0; j < WIDTH ; j++) {
            if (nodes[j + i*WIDTH].edges.size() == 0) {
                obstacles.push_back(&nodes[j + i*WIDTH]);
            }
        }
    }
    return obstacles;
}


// Define the heuristic of the current node (just Euclidian distance in our case
float heuristic(Node current_node, Node target_node) {
    return sqrt(pow(((float)current_node.x-(float)target_node.x), 2) + pow(((float)current_node.y-(float)target_node.y), 2));
}


// Return total cost of path through node n ( f(n)=g(n)+h(n)
float f(Node* n, std::unordered_map<Node*, float> costs, Node target) {
    float g = costs[n];
    float h = heuristic(*n, target);
    return g + h;
}


// to compare the f value of nodes for a priority_queue in A*
struct NodeComparator {
    std::unordered_map<Node*, float>* costs;
    Node targetNode;

    NodeComparator(std::unordered_map<Node*, float>* costs, Node targetNode) : costs(costs), targetNode(targetNode) {}

    bool operator()(Node* node1, Node* node2) {
        return f(node1, *costs, targetNode) > f(node2, *costs, targetNode);
    }
};

// this gets the candidate to visit in the open list (lowest f(n)=g(n)+h(n)
Node* getCurrent(std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open, std::unordered_map<Node*, float> costs, Node target) {
    /*
    auto current = std::min_element(open.begin(), 
                                    open.end(), 
                                    [costs, target](Node* candidate1, Node* candidate2) { 
                                        return f(candidate1, costs, target) < f(candidate2, costs, target);
                                    });
    */
    return open.top();
}


// add neighbors to current node to open list
std::unordered_map<Edge*, Node*> getNeighbors(Node* node) {
    std::unordered_map<Edge*, Node*> neighbors;
    std::for_each(node->edges.begin(), 
                  node->edges.end(),
                  [&neighbors](Edge& edge) { neighbors[&edge] = edge.nodes.second; });
    return neighbors;
}


// return nodes of path
vector<Node*> retrievePath(Node* start, Node* end, std::unordered_map<Node*, Node*> parentDict) {
    vector<Node*> path;
    // we start from the end and add it to the path
    Node* current = end;
    path.push_back(current);
    while(!(*current == *start)) {
        // retrace until the starting node using the parent of the current node
        current = parentDict[current];
        path.push_back(current);
    }
    return path;
}


// The A* algorithm
vector<Node*> Astar(sf::Vector2i startNode, sf::Vector2i endNode) {
    vector<Node*> path;
    // initialize open and close list (open are candidates, close are already visited), a parent dictionary, and an initial cost
    std::unordered_map<Node*, Node*> parentDict;
    std::unordered_map<Node*, float> costs = initCost(getNode(startNode));
    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open(NodeComparator(&costs, *getNode(endNode)));
    // to test if a node is in open or close
    vector<Node*> open_list;
    vector<Node*> closed;
    // set start node as current to explore and get its neighbors
    Node* current = getNode(startNode);
    open.push(current);
    open_list.push_back(current);
    // until open is empty 
    while(open.size() != 0) {
        cout << endl << endl;
        // current to visit is lowest f(n) among open list
        current = getCurrent(open, costs, *getNode(endNode));
        cout << "open size : " << open.size() << " ; closed size : " << closed.size() << endl;
        cout << "current is : " << current->x << " " << current->y << endl;
        cout << costs[current] << " + " << heuristic(*current, *getNode(endNode)) << " = " << f(current, costs, *getNode(endNode)) << endl;
        if (current == getNode(endNode)) {
            cout << "Retrieving path" << endl;
            // END ALGORITHM AND RETRACE PATH
            return retrievePath(getNode(startNode), getNode(endNode), parentDict);
        }
        // get the neighborings nodes and edges of current
        std::unordered_map<Edge*, Node*> neighbors = getNeighbors(current);
        // current has been visited so we put it out of open and place it in closed
        closed.push_back(current);
        open_list.erase(std::remove(open_list.begin(), open_list.end(), current));
        open.pop();
        for (const auto& neighbor : neighbors) {
            // check if neighbor node (the second member in the neighbor pair is node) is in the open and closed list
            bool is_in_open = (std::find(open_list.begin(), open_list.end(), neighbor.second) != open_list.end());
            bool is_in_closed = (std::find(closed.begin(), closed.end(), neighbor.second) != closed.end());
            // compute tentative g_value of neighbor (current cost + edge cost)
            float g_value = costs[current] + neighbor.first->cost;
            // The neighbor has to be skipped if in closed (already explored) and g_value doesn't need update
            if(is_in_closed && g_value >= costs[neighbor.second]) break;
            // update parent and cost of neighbor if never visited or better g_value than its cost
            if(!(is_in_open) || g_value <= costs[neighbor.second]) {
                parentDict[neighbor.second] = current;
                costs[neighbor.second] = g_value;
                // add neighbor in explorable nodes if it wasn't
                if(!is_in_open) {
                    open_list.push_back(neighbor.second);
                    open.push(neighbor.second);
                }
            }
            cout << "nei : " << neighbor.second->x << " " << neighbor.second->y << " : " << f(neighbor.second, costs, *getNode(endNode)) << endl;
        }
        cout << "open size : " << open.size() << endl;
        cout << "close size : " << closed.size() << endl;
    }
    cout << "There is nothing we can do..." << endl;
    return path;
}



int main() {

    // create the grid of nodes
    createGrid();


    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "A*");

    // A* algorithm
    sf::Vector2i START = sf::Vector2i(0, 0);
    sf::Vector2i END = sf::Vector2i(99, 99);

    // A* (chrono to mesure time complexity)
    auto start = std::chrono::high_resolution_clock::now();
    vector<Node*> path = Astar(START, END);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cout << "Time taken by A*: " << duration.count() << " microseconds" << std::endl;
    cout << "Size of path : " << path.size() << endl;
    
    cout << "Drawing..." << endl;
    while (window.isOpen()) {
        // clear window
        window.clear(sf::Color::Black);

        // Draw nodes in path
        sf::CircleShape pathPixel(1);
        pathPixel.setFillColor(sf::Color::White);
        for(Node* node : path) {
            sf::Vector2f nodeCoords = sf::Vector2f(node->x, node->y);
            pathPixel.setPosition(nodeCoords);
            window.draw(pathPixel);
        }

        // Draw starting and end points
        sf::CircleShape startEnd(3);
        startEnd.setFillColor(sf::Color::Red);
        // draw then in window
        startEnd.setPosition(sf::Vector2f(START));
        window.draw(startEnd);
        startEnd.setPosition(sf::Vector2f(END));
        window.draw(startEnd);


        // Draw pixels
        window.display();

    }
}
