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

        for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; j < grid.getSize().y; j++) {
                // Convierto el i j a la posicion
                auto index = Index2(i, j);
                auto pos = grid.getCellPos(index);
                // pos es x, y del espacio
                auto ux0 = velocityX[index];
                auto ux1 = velocityX[right(index)];
                
                auto uy0 = velocityY[index];
                auto uy1 = velocityY[up(index)];

                auto v = lerp(uy0, uy1, 0.5f);
                auto u = lerp(ux0, ux1, 0.5f);

                // Calculo la posicion antigua
                auto oldPos = Vector2(pos.x - (dt * u), pos.y - (dt * v));

                // Interpolo el valor entre los 4 puntos circundantes
                auto oldCell = grid.getCellIndex(oldPos);
                auto oldIndex = Index2(oldCell.x, oldCell.y);
                auto oldCellPos = grid.getCellPos(oldIndex);
                Vector3 newVal;
                if (oldCellPos.x == oldPos.x && oldCellPos.y == oldPos.y) {
                    // Si despues de aplicar la velocidad a la inversa, no he cambiado de posicion, el color es el mismo que tenia
                    newVal = inkCopy[index];
                } else {
                    Index2 aa, ab, ba, bb;
                    if (oldPos.x > oldCellPos.x && oldPos.y > oldCellPos.y) {
                        aa = oldIndex;
                        ba = right(oldIndex);
                        ab = up(oldIndex);
                        bb = right(up(oldIndex));
                    } else if (oldPos.x > oldCellPos.x && oldPos.y < oldCellPos.y) {
                        aa = down(oldIndex);
                        ba = left(down(oldIndex));
                        ab = oldIndex;
                        bb = right(oldIndex);
                    } else if (oldPos.x < oldCellPos.x && oldPos.y > oldCellPos.y) {
                        aa = left(oldIndex);
                        ba = oldIndex;
                        ab = left(down(oldIndex));
                        bb = up(oldIndex);
                    } else {
                        aa = left(down(oldIndex));
                        ba = down(oldIndex);
                        ab = left(oldIndex);
                        bb = oldIndex;
                    }

                    auto aaPos = grid.getCellPos(aa);
                    float tx = oldPos.x - aaPos.x;
                    tx = clamp(tx, 0, 1);
                    float ty = oldPos.y - aaPos.y;
                    ty = clamp(ty, 0, 1);

                    auto inkAA = inkCopy.getValue(aa);
                    auto inkBA = inkCopy.getValue(ba);
                    auto inkAB = inkCopy.getValue(ab);
                    auto inkBB = inkCopy.getValue(bb);

                    newVal = bilerp(inkAA, inkBA, inkAB, inkBB, tx, ty);
                }
                
                inkRGB[index] = newVal;                
                // Primero obtengo la velocidad en esta posicion
                // interpolar los valores, sumar los valores y dividir entre dos por cada eje 

                // Una vez tengo los valores, ya tengo u, y v para hacer la posicion en x(t - dt)

                // Ahora x es posicion del espacio, hay que convertir a indices del grid                
            }
        }
    }

    {        
        // Igual que lo otro
        //for (int i = 0; i < grid.getSize().x; i++) {
        //    for (int j = 0; i < grid.getSize().y; j++) {
        //        // Convierto el i j a la posicion
        //        auto pos = grid.getCellPos(Index2(i, j));
        //        // pos es x, y del espacio

        //        // Primero obtengo la velocidad en esta posicion
        //        // interpolar los valores, sumar los valores y dividir entre dos por cada eje
        //    }
        //}
        // Velocity acvection term HERE
        // x(t − dt) = x(t) + dt * (−u)
        /*for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; i < grid.getSize().y; j++) {
                auto pos = grid.getCellPos(Index2(i, j));                
                auto prevX = i + dt * getVelocityX();

                         auto nuevoPos = pos + dt * (-v)
            }
        }*/

        auto uCopy = Array2<float>(velocityX);
        auto vCopy = Array2<float>(velocityY);        
        
        for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; j < grid.getSize().y; j++) {
                // Convierto el i j a la posicion
                auto index = Index2(i, j);
                auto uPos = grid.getFaceXPos(index);
                auto vPos = grid.getFaceYPos(index);

                // pos es x, y del espacio
                auto ux0 = uCopy[index];
                auto ux1 = uCopy[right(index)];

                auto uy0 = vCopy[index];
                auto uy1 = vCopy[up(index)];

                auto v = lerp(uy0, uy1, 0.5f);
                auto u = lerp(ux0, ux1, 0.5f);

                // Calculo la posicion antigua
                auto oldUPos = Vector2(uPos.x - (dt * u), uPos.y - (dt * v));
                auto oldVPos = Vector2(vPos.x - (dt * u), vPos.y - (dt * v));

                // Interpolo el valor entre los 4 puntos circundantes
                auto oldUCell = grid.getFaceIndex(oldUPos, 0);                
                auto oldVCell = grid.getFaceIndex(oldVPos, 1);

                auto oldUIndex = Index2(oldUCell.x, oldUCell.y);
                auto oldVIndex = Index2(oldVCell.x, oldVCell.y);

                auto oldUCellPos = grid.getFaceXPos(oldUIndex);
                auto oldVCellPos = grid.getFaceYPos(oldVIndex);

                Index2 aaU, abU, baU, bbU;
                Index2 aaV, abV, baV, bbV;
                float newUVal, newVVal;

                if (uPos.x == oldUPos.x && uPos.y == oldUPos.y) {
                    newUVal = uCopy[index];
                } else {
                    if (oldUPos.x > oldUCellPos.x && oldUPos.y > oldUCellPos.y) {
                        aaU = oldUIndex;
                        baU = right(oldUIndex);
                        abU = up(oldUIndex);
                        bbU = right(up(oldUIndex));
                    } else if (oldUPos.x > oldUCellPos.x && oldUPos.y < oldUCellPos.y) {
                        aaU = down(oldUIndex);
                        baU = left(down(oldUIndex));
                        abU = oldUIndex;
                        bbU = right(oldUIndex);
                    } else if (oldUPos.x < oldUCellPos.x && oldUPos.y > oldUCellPos.y) {
                        aaU = left(oldUIndex);
                        baU = oldUIndex;
                        abU = left(down(oldUIndex));
                        bbU = up(oldUIndex);
                    } else {
                        aaU = left(down(oldUIndex));
                        baU = down(oldUIndex);
                        abU = left(oldUIndex);
                        bbU = oldUIndex;
                    }

                    auto aaUPos = grid.getCellPos(aaU);
                    float txU = oldUPos.x - aaUPos.x;
                    txU = clamp(txU, 0, 1);
                    float tyU = oldUPos.y - aaUPos.y;
                    tyU = clamp(tyU, 0, 1);

                    auto uAA = uCopy[aaU];
                    auto uBA = uCopy[baU];
                    auto uAB = uCopy[abU];
                    auto uBB = uCopy[bbU];

                    newUVal = bilerp(uAA, uBA, uAB, uBB, txU, tyU);
                }
                velocityX[index] = newUVal;

                if (vPos.x == oldVPos.x && vPos.y == oldVPos.y) {
                    newVVal = vCopy[index];
                } else {
                    if (oldVPos.x > oldVCellPos.x && oldVPos.y > oldVCellPos.y) {
                        aaV = oldVIndex;
                        baV = right(oldVIndex);
                        abV = up(oldVIndex);
                        bbV = right(up(oldVIndex));
                    } else if (oldVPos.x > oldVCellPos.x && oldVPos.y < oldVCellPos.y) {
                        aaV = down(oldVIndex);
                        baV = left(down(oldVIndex));
                        abV = oldVIndex;
                        bbV = right(oldVIndex);
                    } else if (oldVPos.x < oldVCellPos.x && oldVPos.y > oldVCellPos.y) {
                        aaV = left(oldVIndex);
                        baV = oldVIndex;
                        abV = left(down(oldVIndex));
                        bbV = up(oldVIndex);
                    } else {
                        aaV = left(down(oldVIndex));
                        baV = down(oldVIndex);
                        abV = left(oldVIndex);
                        bbV = oldVIndex;
                    }

                    auto aaVPos = grid.getCellPos(aaV);
                    float txV = oldVPos.x - aaVPos.x;
                    txV = clamp(txV, 0, 1);
                    float tyV = oldVPos.y - aaVPos.y;
                    tyV = clamp(tyV, 0, 1);

                    auto vAA = vCopy[aaV];
                    auto vBA = vCopy[baV];
                    auto vAB = vCopy[abV];
                    auto vBB = vCopy[bbV];

                    newVVal = bilerp(vAA, vBA, vAB, vBB, txV, tyV);
                }
                velocityY[index] = newVVal;                
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
        /*for (uint i = 2; i < inkRGB.getSize().x / 4; ++i)
            for (uint j = 2; j < inkRGB.getSize().y / 4; ++j)
                inkRGB[Index2(i, j)] = Vector3(1, 1, 0);

        Array2<float> &u = velocityX;
        for (uint i = 0; i < u.getSize().x; ++i)
            for (uint j = 0; j < u.getSize().y; ++j)
                u[Index2(i, j)] = 2.0f;

        Array2<float> &v = velocityY;
        for (uint i = 0; i < v.getSize().x; ++i)
            for (uint j = 0; j < v.getSize().y; ++j)
                v[Index2(i, j)] = 2.0f;*/

        inkRGB[Index2(0, 0)] = Vector3(1, 0, 1);
        velocityX[Index2(0, 0)] = 0.0f;
        velocityY[Index2(0, 0)] = 1.0f;
    }
}

void Fluid2::fluidVolumeForces(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Gravity term HERE

        // Aplicamos la formula de la diapositiva 31, solo hay que actualizar las velocidades en y
        // velocidad en y = velicidad en n * dt * -9.81
        /*auto vCopy = Array2<float>(getVelocityY());
        for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; j < grid.getSize().y; j++) {
                auto index = Index2(i, j);
                velocityY[index] = vCopy[index] * dt * -Scene::kGravity;
            }
        }*/

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
        /*auto uCopy = Array2<float>(getVelocityX());
        auto vCopy = Array2<float>(getVelocityY());

        for (int i = 0; i < grid.getSize().x; i++) {
            for (int j = 0; j < grid.getSize().y; j++) {
                auto index = Index2(i, j);
                auto u = uCopy[index];
                auto v = vCopy[index];

                auto fooU =
                    (uCopy[right(index)] - (2 * u) + uCopy[left(index)]) / grid.getCellDx().x * grid.getCellDx().x;
                auto fooV =
                    (vCopy[right(index)] - (2 * v) + vCopy[left(index)]) / grid.getCellDx().y * grid.getCellDx().y;
                auto newU = u * (dt / Scene::kDensity) * Scene::kViscosity * fooU;
                auto newV = v * (dt / Scene::kDensity) * Scene::kViscosity * fooV;

                velocityX[index] = newU;
                velocityY[index] = newV;
            }
        }*/
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
Index2 Fluid2::left(Index2 &index)
{
    return Index2(clamp(index.x - 1, 0, index.x - 1), index.y);
}

Index2 Fluid2::right(Index2 &index)
{
    return Index2(clamp(index.x + 1, index.x + 1, grid.getSize().x), index.y);
}
Index2 Fluid2::up(Index2 &index)
{
    return Index2(index.x, clamp(index.y + 1, index.y + 1, grid.getSize().y));
}
Index2 Fluid2::down(Index2 &index)
{
    return Index2(index.x, clamp(index.y - 1, 0, index.y - 1));
}
}  // namespace asa
