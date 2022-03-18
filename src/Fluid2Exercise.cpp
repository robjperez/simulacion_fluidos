#include "Scene.h"

#include "Numeric/PCGSolver.h"

namespace asa
{
namespace
{
////////////////////////////////////////////////
// Add any reusable classes or functions HERE //
////////////////////////////////////////////////
}  // namespace

// advection
void Fluid2::fluidAdvection(const float dt)
{
    {
        // Ojo que la rejilla esta desplazada

        
        // Ink advecion HERE

        // Primero nos copiamos toda la tinta a otro Array2
        // Para los calculos usamos esta copia y actualizamos en la buena
        // Recorremos todas las celdillas
        auto inkCopy = Array2<asa::Vector3>(inkRGB);

        auto &u = getVelocityX();
        auto &v = getVelocityY();
        for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; i < grid.getSize().y; j++) {
                // Convierto el i j a la posicion
                auto pos = grid.getCellPos(Index2(i, j));
                // pos es x, y del espacio
                auto ux = u[Index2(pos.x, pos.y)];
                auto ux1 = u[Index2(pos.x+1, pos.y)];
                auto inter = ux + ux1 / 2;

                
                // Primero obtengo la velocidad en esta posicion
                // interpolar los valores, sumar los valores y dividir entre dos por cada eje 

                // Una vez tengo los valores, ya tengo u, y v para hacer la posicion en x(t - dt)

                // Ahora x es posicion del espacio, hay que convertir a indices del grid

                
            }
        }
    }

    {        
        // Igual que lo otro
        for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; i < grid.getSize().y; j++) {
                // Convierto el i j a la posicion
                auto pos = grid.getCellPos(Index2(i, j));
                // pos es x, y del espacio

                // Primero obtengo la velocidad en esta posicion
                // interpolar los valores, sumar los valores y dividir entre dos por cada eje
            }
        }
        // Velocity acvection term HERE
        // x(t − dt) = x(t) + dt * (−u)
        for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; i < grid.getSize().y; j++) {
                auto pos = grid.getCellPos(Index2(i, j));                
                auto prevX = i + dt * getVelocityX();

                         auto nuevoPos = pos + dt * (-v)
            }
        }
    }
}

void Fluid2::fluidEmission()
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Emitters contribution HERE

        // Aqui configuramos los emisores donde queramos

        // Basicamente es poner donde queramos unos bounding boxes que metan velocidades y tintas
        // Ver la funcion de Scene donde se genera unos emisores de prueba
    }
}

void Fluid2::fluidVolumeForces(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Gravity term HERE

        // Aplicamos la formula de la diapositiva 31, solo hay que actualizar las velocidades en y
        // velocidad en y = velicidad en n * dt * -9.81
    }
}

void Fluid2::fluidViscosity(const float dt)
{
    // Aplicamos la foruma de la diapositiva 28
    // velocidad nueva en ij = velocidad en ij + dt / densidad * viscosidad * (dif finitas de las velocidades)
    
    // OJo si tenemos que crear una copia del array para que los datos que calculemos esten basados en el paso anterior
    // Array tiene copia implementada
    if (Scene::testcase >= Scene::SMOKE) {
        // Viscosity term HERE
    }
}

void Fluid2::fluidPressureProjection(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Incompressibility / Pressure term HERE
        // Primero calculamos el vector derecho, que son las divergencia de las velocidades
        // Con los laterales que dan a un solido, Por cada pared, las velocidades de la pared las ponemos a 0
        // El suelo es solido, asi que todas las velocidades del suelo las pongo a 0. las velocidades del suelo a 0, son las velocidades que tienen la y a 0
        // lo mejor es tratar como condiciones de contorno solidos por los 4 lados, asi que ponemos a 0 las velocidades todos los bordes

        // Calculamos el vector derecho
        // Me creo un vector del tamaño de las incognitas, osea 100x100, 10.000 double
        // auto b = new Vector<double>(10000)
        // Metemos en cada elemento la divergencia de la velocidad por la constante

        // Ahora construyo la matrix de coeficientes
        // new MatrixDispersa de 10.000 x 10.000
        // Relleno los elementos de matriz con el esquema de la pagina 18 de las diapositivas. Matrix de double tambien

        // Con matrix y vector b, me creo un PCGSolver para resolver el sistema, antes me creo el vector de incognitas, 10.000, y lo inicializamos a 0
        // igual merece la pena bajar a 25x25 para debuggear

        // Llamo a solve y ya tengo el vector de p

        // Con las presiones resueltas, tengo que actualizar las velocidades
        // Primero meto el vector de p, en la variable pressure que es un campo 2d de los valores
        // El array tiene un getData, que es un puntero y podemos recorrerlo de i a 10.000 para actualizar todos los valores

        // Parte final es diapositiva 19
        // Por cada velocidad tanto en x como en y, las actualizo usando la formula de la pag 19
        // Cuidado con las condiciones de contorno. Clampeo los indices, -1 pasa a 0 o 101 a 100
        // nueva velocidad = velocidad antigua - dt / densidad * Pi+1,j - Pi,j / dx

    }
}
}  // namespace asa
