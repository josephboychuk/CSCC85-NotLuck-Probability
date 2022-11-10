# Compiles with -g and -01 for Valgrind
g++ -c -g -O1 ParticleFilters.c
g++  *.o -O1 -g -lGL -lGLU -lglut -no-pie -o ParticleFilters

