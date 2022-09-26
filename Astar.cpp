#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <map>

using namespace std;
using namespace Eigen;

const int width = 1024;
const int height = 960;


struct edge {
	Eigen::Vector2i node;
	float cost;
};


std::map<Eigen::Vector2i, std::vector<edge>> graph;

void createGrid() {
	//insert nodes
	for (int i = 0 ; i < width; i++) {
		for (int j = 0 ; j < height; j++) {
			graph.insert({Eigen::Vector2i(i, j), std::vector<edge>()});
		}
	}
	//add edges to nodes
	for (int i = 0 ; i < width ; i++) {
		for (int j = 0 ; j < height ; j++) {
			//exploring neighborhood
			for (int m = -1 ; m <= 1 ; m++) {
				for (int n = -1 ; n <= 1 ; n++) {
					//not adding edge to self or out of bound
					if(m != 0 && n != 0 && (i+m) >= 0 && (i+m) < width && (j+n) >=0 && (j+n) < height) {
						//check whether neighbor is diagonal or not
						edge e;
						e.node = Eigen::Vector2i(i+m, j+n);
						(m == 0 || n == 0) ? e.cost = 1 : e.cost = std::sqrt(2);
						graph[Eigen::Vector2i(i,j)].push_back(e);
					}
				}
			}
		}
	}
}


int main() {
	createGrid();
}
