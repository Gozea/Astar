#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;

const int width = 1024;
const int height = 960;

class Node;
class Edge;
class Graph;

class Node {
	private:
		vector<Edge *> edges;
		Vector2f coord;
	public:
		Node(int x, int y) {
			this->coord.x() = x;
			this->coord.y() = y;
		}


		bool operator==(Node n) { return tie(this->coord.x(), this->coord.y()) == tie(n.getCoords().x(), n.getCoords().y());}

		Vector2f getCoords() {return this->coord;}

		void show() { 
			cout << "coords" << endl;
			cout << getCoords().x() << " " << getCoords().y() << endl;
			for (Edge* e : edges) {
				(*e).show();
			}
			cout << endl;
		}
		//void add_edge(Edge e);
		//vector<*Edge> getEdges();
		//Node getNodeThroughEdge(Edge e);
};

class Edge {
	private:
		Node* n[2];
		int cost = 1;
	public:
		Edge(Node* n1, Node* n2) {
			this->n[0] = n1;
			this->n[1] = n2;
		}


		Node getNeighbourOf(Node node) {
			if (node==*n[0]) return *n[1];
			if (node==*n[1]) return *n[0];
			return Node(-1, -1);
		}

		void show() {
			cout << "node 0" << endl;
			n[0]->show();
			cout << "node 1" << endl;
			n[1]->show();
		}
};

class Graph {
	private :
		vector<Node*> nodes;
		vector<Edge*> edges;	
	public:
		void createGrid() {
			for (int i = 0 ; i < height; i++) {
				for (int j = 0 ; j < width; j++) {
					nodes.push_back(new Node(j, i));
				}
			}
				
			for (int i = 0 ; i < height ; i++) {
				for (int j = 0 ; j < width ; j++) {
					//edge with the node on right
					edges.push_back(new Edge(this->nodes[j+i*height], this->nodes[(j+1)+i*height]));
					//edge with node bellow 
					edges.push_back(new Edge(this->nodes[j+i*height], this->nodes[j+(i+1)*height]));
					//edge with node right-bellow
					edges.push_back(new Edge(this->nodes[j+i*height], this->nodes[(j+1)+(i+1)*height]));
				}
			}
		}


		void show() {
			for (int i = 0 ; i < 3 ; i++) {
				for (int j = 0 ; j < 3 ; j++) {
					(this->nodes[j+i*width])->show();  
				}
			}
		}

		void clear() {
			for (Node* n : nodes) {
				delete n;
			}
			nodes.clear();
			for (Edge* e : edges) {
				delete e;
			}
			edges.clear();
		}
};



int main() {
	Graph G;
	G.createGrid();
	G.show();
	G.clear();
}
