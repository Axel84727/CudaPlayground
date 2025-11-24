Como compilar con Make(basico)
siempre que se quiera compilar con make es buena practica crear un directorio
mkdir /build
cd build
cada vez que cambie el archivo CMakeLists.txt debes ejecutar 'cmake ..'
y luego ejecutar 'make' para que el proyecto se construya

y para compilarlo ya
./NombreDelEjecutable
