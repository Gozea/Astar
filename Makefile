LIBS = -lsfml-graphics -lsfml-window -lsfml-system


exec: Astar
	./Astar

Astar: Astar.o
	g++ Astar.o -o Astar $(LIBS)

Astar.o: Astar.cpp
	g++ -c Astar.cpp
