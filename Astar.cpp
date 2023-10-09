#include <iostream>
#include <SFML/Graphics.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;

const int WIDTH = 920;
const int HEIGHT = 480;

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
            nodes[j + i*HEIGHT].x = j;
            nodes[j + i*HEIGHT].y = i;
			//exploring neighborhood
			for (int m = -1 ; m <= 1 ; m++) {
				for (int n = -1 ; n <= 1 ; n++) {
					//not adding edge to self or out of bound
					if(m != 0 && n != 0 && (i+m) >= 0 && (i+m) < HEIGHT && (j+n) >=0 && (j+n) < WIDTH) {
						//check whether neighbor is diagonal or not
						float c = (m == 0 || n == 0) ? c = 1 : c = std::sqrt(2);
                        // create the edge and add it
                        Edge e(&nodes[j + i*HEIGHT], &nodes[j+n + (i+m)*HEIGHT], c);
                        nodes[j + i*HEIGHT].edges.push_back(e);
					}
				}
			}
		}
	}
}

// return node from cooordinates
Node* getNode(int x, int y) {
    return &nodes[y + x*HEIGHT];
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
            if (nodes[j + i*HEIGHT].edges.size() != 0) size++;
        }
    }
    return size;
}

// Return vector of isolated nodes
vector<Node*> getObstacles() {
    vector<Node*> obstacles;
    for (int i=0; i < HEIGHT ; i++) {
        for (int j=0; j < WIDTH ; j++) {
            if (nodes[j + i*HEIGHT].edges.size() == 0) {
                obstacles.push_back(&nodes[j + i*HEIGHT]);
            }
        }
    }
    return obstacles;
}




int main() {

    createGrid();
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "A*");

    while (window.isOpen()) {
        window.clear(sf::Color::Black);

        window.display();

    }

}
