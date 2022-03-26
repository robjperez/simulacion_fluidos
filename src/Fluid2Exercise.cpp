#include "Scene.h"

#include "Numeric/PCGSolver.h"

namespace asa
{
namespace
{
    // TODO!!!
    // REMOVE THIS FUNCTION!
float check(Index2 &a, Array2<float> &b)
{
    Index2 sz = b.getSize();
    if (a.x >= 0 && a.x < sz.x && a.y >= 0 && a.y < sz.y)
        return b[a];
    else
        return 0.0f;
}

template<class T>
T value_or_zero(Index2 &index, Array2<T> &data)
{
    auto &dataSize = data.getSize();
    if (index.x < 0 || 
        index.x > (dataSize.x - 1) || 
        index.y < 0 || 
        index.y > (dataSize.y - 1))
    {
        return 0.0f;
    }
    return data[index];
}


////////////////////////////////////////////////
// Add any reusable classes or functions HERE //
////////////////////////////////////////////////
}  // namespace


Index2 Fluid2::left(Index2 &index)
{
    return Index2(clamp(index.x - 1, 0, grid.getSize().x - 1), index.y);
}
Index2 Fluid2::right(Index2 &index)
{
    return Index2(clamp(index.x + 1, 0, grid.getSize().x - 1), index.y);
}
Index2 Fluid2::up(Index2 &index)
{
    return Index2(index.x, clamp(index.y + 1, 0, grid.getSize().y - 1));
}
Index2 Fluid2::down(Index2 &index)
{
    return Index2(index.x, clamp(index.y - 1, 0, grid.getSize().y - 1));
}

// advection
void Fluid2::fluidAdvection(const float dt)
{
    auto minPos = grid.getDomain().minPosition;
    auto maxPos = grid.getDomain().maxPosition;
    {
        // Ojo que la rejilla esta desplazada

        // Ink advecion HERE

        // Primero nos copiamos toda la tinta a otro Array2
        // Para los calculos usamos esta copia y actualizamos en la buena
        // Recorremos todas las celdillas
        auto inkCopy = Array2<asa::Vector3>(inkRGB);
        auto size = grid.getSize();        

        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                // Convierto el i j a la posicion
                auto index = Index2(i, j);
                auto pos = grid.getCellPos(index);
                // pos es x, y del espacio

                auto u = (velocityX[index] + velocityX[right(index)]) / 2;
                auto v = (velocityY[index] + velocityY[up(index)]) / 2;

                // Calculo la posicion antigua. 
                auto oldPos = pos - (dt * Vector2(u, v));

                // Clampeamos por si nos salimos
                oldPos = Vector2(
                    clamp(oldPos.x, minPos.x, maxPos.x),                     
                    clamp(oldPos.y, minPos.y, maxPos.y)
                );

                // Interpolo el valor entre los 4 puntos circundantes
                auto oldCellIndex = grid.getCellIndex(oldPos);
                
                auto oldStartPos = Vector2(
                    floorf(oldCellIndex.x),
                    floorf(oldCellIndex.y)
                );
                auto oldEndPos = Vector2(
                    ceilf(oldCellIndex.x),
                    ceilf(oldCellIndex.y)
                );

                // t nos va a dar el punto de interpolacion
                auto t = oldCellIndex - oldStartPos;

                // Sacamos los 4 puntos
                auto inkAA = inkCopy[Index2(
                    clamp(oldStartPos.x, 0, size.x - 1), 
                    clamp(oldStartPos.y, 0, size.y - 1))];
                auto inkBA = inkCopy[Index2(
                    clamp(oldEndPos.x, 0, size.x - 1), 
                    clamp(oldStartPos.y, 0, size.y - 1))];
                auto inkAB = inkCopy[Index2(
                    clamp(oldStartPos.x, 0, size.x - 1), 
                    clamp(oldEndPos.y, 0, size.y - 1))];
                auto inkBB = inkCopy[Index2(
                    clamp(oldEndPos.x, 0, size.x - 1), 
                    clamp(oldEndPos.y, 0, size.y - 1))];                

                auto newVal = bilerp(inkAA, inkBA, inkAB, inkBB, t.x, t.y);                
                inkRGB[index] = newVal;
                // Primero obtengo la velocidad en esta posicion
                // interpolar los valores, sumar los valores y dividir entre dos por cada eje

                // Una vez tengo los valores, ya tengo u, y v para hacer la posicion en x(t - dt)

                // Ahora x es posicion del espacio, hay que convertir a indices del grid
            }
        }
    }

    {
        auto uCopy(velocityX);
        auto vCopy(velocityY);
        auto sizeU = velocityX.getSize();
        auto sizeV = velocityY.getSize();

        // Transportamos las velocidades en X
        for (int i = 0; i < sizeU.x; i++) {
            for (int j = 0; j < sizeU.y; j++) {
                // Convierto el i j a la posicion
                auto index = Index2(i, j);

                // Tenemos que calcular v para ir a la posicion antigua
                auto uPos = grid.getFaceXPos(index);
                
                auto u = uCopy[index];
                // Interpolamos las velocidades en y
                auto v0 = vCopy[Index2(clamp(i - 1, 0, sizeV.x - 1), clamp(j, 0, sizeV.y - 1))];
                auto v1 = vCopy[Index2(clamp(i, 0, sizeV.x - 1), clamp(j, 0, sizeV.y - 1))];
                auto v2 = vCopy[Index2(clamp(i - 1, 0, sizeV.x - 1), clamp(j + 1, 0, sizeV.y - 1))];
                auto v3 = vCopy[Index2(clamp(i, 0, sizeV.x - 1), clamp(j + 1, 0, sizeV.y - 1))];
                auto v = (v0 + v1 + v2 + v3) / 4;

                auto oldPos = uPos - dt * Vector2(u, v);               
                           
                // Interpolo el valor entre los 4 puntos circundantes
                auto oldCellIndex = grid.getFaceIndex(oldPos, 0);
                auto oldStartPos = Vector2(
                    floorf(oldCellIndex.x), 
                    floorf(oldCellIndex.y));
                auto oldEndPos = Vector2(
                    ceilf(oldCellIndex.x), 
                    ceilf(oldCellIndex.y));
                auto t = oldCellIndex - oldStartPos;
                
                auto uAA = uCopy[Index2(clamp(oldStartPos.x, 0, sizeU.x - 1), clamp(oldStartPos.y, 0, sizeU.y - 1))];
                auto uBA = uCopy[Index2(clamp(oldEndPos.x, 0, sizeU.x - 1), clamp(oldStartPos.y, 0, sizeU.y - 1))];
                auto uAB = uCopy[Index2(clamp(oldStartPos.x, 0, sizeU.x - 1), clamp(oldEndPos.y, 0, sizeU.y - 1))];
                auto uBB = uCopy[Index2(clamp(oldEndPos.x, 0, sizeU.x - 1), clamp(oldEndPos.y, 0, sizeU.y - 1))];

                auto newVal = bilerp(uAA, uBA, uAB, uBB, t.x, t.y);
                velocityX[index] = newVal;
            }
        }

        // Transportamos las velocidades en Y
        for (int i = 0; i < sizeV.x; i++) {
            for (int j = 0; j < sizeV.x; j++) {
                // Convierto el i j a la posicion
                auto index = Index2(i, j);

                // Tenemos que calcular v para ir a la posicion antigua
                auto vPos = grid.getFaceYPos(index);

                auto v = vCopy[index];
                // Interpolamos las velocidades en y
                auto u0 = uCopy[Index2(clamp(i, 0, sizeU.x - 1), clamp(j - 1, 0, sizeU.y - 1))];
                auto u1 = uCopy[Index2(clamp(i, 0, sizeU.x - 1), clamp(j, 0, sizeU.y - 1))];
                auto u2 = uCopy[Index2(clamp(i + 1, 0, sizeU.x - 1), clamp(j - 1, 0, sizeU.y - 1))];
                auto u3 = uCopy[Index2(clamp(i + 1, 0, sizeU.x - 1), clamp(j, 0, sizeU.y - 1))];
                auto u = (u0 + u1 + u2 + u3) / 4;

                auto oldPos = vPos - dt * Vector2(u, v);

                // Interpolo el valor entre los 4 puntos circundantes
                auto oldCellIndex = grid.getFaceIndex(oldPos, 1);
                auto oldStartPos = Vector2(floorf(oldCellIndex.x), floorf(oldCellIndex.y));
                auto oldEndPos = Vector2(ceilf(oldCellIndex.x), ceilf(oldCellIndex.y));
                auto t = oldCellIndex - oldStartPos;

                auto vAA = vCopy[Index2(clamp(oldStartPos.x, 0, sizeV.x - 1), clamp(oldStartPos.y, 0, sizeV.y - 1))];
                auto vBA = vCopy[Index2(clamp(oldEndPos.x, 0, sizeV.x - 1), clamp(oldStartPos.y, 0, sizeV.y - 1))];
                auto vAB = vCopy[Index2(clamp(oldStartPos.x, 0, sizeV.x - 1), clamp(oldEndPos.y, 0, sizeV.y - 1))];
                auto vBB = vCopy[Index2(clamp(oldEndPos.x, 0, sizeV.x - 1), clamp(oldEndPos.y, 0, sizeV.y - 1))];

                auto newVal = bilerp(vAA, vBA, vAB, vBB, t.x, t.y);
                velocityY[index] = newVal;
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
        auto size = grid.getSize();
        for (int i = (size.x / 2) - 4; i < (size.x / 2) + 4; i++) {
        //for (int i = 0; i < 10; i++) {
            for (int j = 5; j < 30; j++) {
            //for (int j = 0; j < 10; j++) {
                auto idx = Index2(i, j);
            if (i < size.x / 2) {
                    inkRGB[idx] = Vector3(1, 0, 1);
            } else {
                inkRGB[idx] = Vector3(1, 1, 0);
            }
                
                velocityY[idx] = 10.0f;
                velocityX[idx] = 0.0f;
            }
        }    
    }
}

void Fluid2::fluidVolumeForces(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Gravity term HERE

        // Aplicamos la formula de la diapositiva 31, solo hay que actualizar las velocidades en y
        // velocidad en y = velicidad en n * dt * -9.81
        
        for (int i = 0; i < grid.getSizeFacesY().x; i++) {
            for (int j = 0; j < grid.getSizeFacesY().y; j++) {
                auto index = Index2(i, j);
                velocityY[index] += dt * Scene::kGravity;  // La gravedad ya esta definida con -
            }
        }
    }
}

void Fluid2::fluidViscosity(const float dt)
{
    // Aplicamos la formula de la diapositiva 28
    // velocidad nueva en ij = velocidad en ij + dt / densidad * viscosidad * (dif finitas de las velocidades)

    // OJo si tenemos que crear una copia del array para que los datos que calculemos esten basados en el paso anterior
    // Array tiene copia implementada
    if (Scene::testcase >= Scene::SMOKE) {
        // viscosity
        Vector2 cellDx = grid.getCellDx();

        Index2 faceSizeX = grid.getSizeFacesX();
        Index2 faceSizeY = grid.getSizeFacesY();

        Array2<float> horizontalV(velocityX.getSize());
        float density = Scene::kDensity;
        float viscosity = Scene::kViscosity;
        for (int i = 0; i < faceSizeX.x; i++) {
            for (int j = 0; j < faceSizeX.y; j++) {
                Index2 p(i, j);

                //	(i,j)
                float V_ij = check(p, velocityX);
                //	(i+1,j)
                float V_right = check(Index2(p.x + 1, p.y), velocityX);
                //	(i-1,j)
                float V_left = check(Index2(p.x - 1, p.y), velocityX);
                //	(i,j+1)
                float V_up = check(Index2(p.x, p.y + 1), velocityX);
                //	(i,j-1)
                float V_down = check(Index2(p.x, p.y - 1), velocityX);

                float value = V_ij + (dt / density) * viscosity *
                                         (((V_right - 2.0f * V_ij + V_left) / (cellDx.x * cellDx.x)) +
                                          ((V_up - 2.0f * V_ij + V_down) / (cellDx.y * cellDx.y)));

                Index2 sz = horizontalV.getSize();
                if (p.x > 0 && p.x < sz.x - 1 && p.y >= 0 && p.y < sz.y) {
                    horizontalV[p] = value;
                }
            }
        }

        Array2<float> verticalV(velocityY.getSize());
        for (int i = 0; i < faceSizeY.x; i++) {
            for (int j = 0; j < faceSizeY.y; j++) {
                Index2 p(i, j);

                //	(i,j)
                float V_ij = check(p, velocityY);
                //	(i+1,j)
                float V_right = check(Index2(p.x + 1, p.y), velocityY);
                //	(i-1,j)
                float V_left = check(Index2(p.x - 1, p.y), velocityY);
                //	(i,j+1)
                float V_up = check(Index2(p.x, p.y + 1), velocityY);
                //	(i,j-1)
                float V_down = check(Index2(p.x, p.y - 1), velocityY);

                float value = V_ij + (dt / density) * viscosity *
                                         (((V_right - 2.0f * V_ij + V_left) / (cellDx.x * cellDx.x)) +
                                          ((V_up - 2.0f * V_ij + V_down) / (cellDx.y * cellDx.y)));

                Index2 sz = verticalV.getSize();
                if (p.x >= 0 && p.x < sz.x && p.y > 0 && p.y < sz.y - 1) {
                    verticalV[p] = value;
                }
            }
        }
        velocityX = horizontalV;
        velocityY = verticalV;
    }

    //if (Scene::testcase >= Scene::SMOKE) {
    //    // Viscosity term HERE
    //    auto uCopy(velocityX);
    //    auto vCopy(velocityY);
    //    auto &sizeU = velocityX.getSize();
    //    auto &sizeV = velocityY.getSize();
    //    auto dx = grid.getCellDx();
    //    auto c = (dt * Scene::kViscosity) / Scene::kDensity;

    //    // Calculamos la parte U del vector
    //    for (int i = 0; i < sizeU.x; i++) {
    //        for (int j = 0; j < sizeU.y; j++) {
    //            auto index = Index2(i, j);
    //            if (i == 0 || j == 0 || i == sizeU.x - 1 || j == sizeU.y - 1) {
    //                velocityX[index] = 0;
    //                continue;
    //            }
    //            
    //            auto u = uCopy[index];

    //            auto rightU = uCopy[Index2(i + 1, j)];                
    //            auto leftU = uCopy[Index2(i - 1, j)];                
    //            auto upU = uCopy[Index2(i, j + 1)];                
    //            auto downU = uCopy[Index2(i, j - 1)];                

    //            auto difU1 = (rightU - 2.0f * u + leftU) / (dx.x * dx.x);
    //            auto difU2 = (upU - 2.0f * u + downU) / (dx.y * dx.y);
    //            auto difU = difU1 + difU2;
    //            
    //            velocityX[index] = u + (c * difU);
    //        }
    //    }

    //    // Calculamos la parte V del vector
    //    for (int i = 0; i < sizeV.x; i++) {
    //        for (int j = 0; j < sizeV.y; j++) {
    //            auto index = Index2(i, j);
    //            if (i == 0 || j == 0 || i == sizeV.x - 1 || j == sizeV.y - 1) {
    //                velocityY[index] = 0;
    //                continue;
    //            }
    //            auto v = vCopy[index];

    //            auto rightV = vCopy[Index2(i + 1, j)];                
    //            auto leftV = vCopy[Index2(i - 1, j)];                
    //            auto upV = vCopy[Index2(i, j + 1)];                
    //            auto downV = vCopy[Index2(i, j - 1)];

    //            auto difV1 = (rightV - 2.0f * v + leftV) / (dx.x * dx.x);
    //            auto difV2 = (upV - 2.0f * v + downV) / (dx.y * dx.y);
    //            auto difV = difV1 + difV2;
    //            
    //            velocityY[index] = v + (c * difV);
    //        }
    //    }            
    //}
}

void Fluid2::fluidPressureProjection(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Incompressibility / Pressure term HERE
        // Primero calculamos el vector derecho, que son las divergencia de las velocidades
        // Con los laterales que dan a un solido, Por cada pared, las velocidades de la pared las ponemos a 0
        // El suelo es solido, asi que todas las velocidades del suelo las pongo a 0. las velocidades del suelo a 0, son
        // las velocidades que tienen la y a 0 lo mejor es tratar como condiciones de contorno solidos por los 4 lados,
        // asi que ponemos a 0 las velocidades todos los bordes

        // Calculamos el vector derecho
        // Me creo un vector del tamaño de las incognitas, osea 100x100, 10.000 double
        // auto b = new Vector<double>(10000)
        // Metemos en cada elemento la divergencia de la velocidad por la constante

        // Ahora construyo la matrix de coeficientes
        // new MatrixDispersa de 10.000 x 10.000
        // Relleno los elementos de matriz con el esquema de la pagina 18 de las diapositivas. Matrix de double tambien

        // Con matrix y vector b, me creo un PCGSolver para resolver el sistema, antes me creo el vector de
        // incognitas, 10.000, y lo inicializamos a 0 igual merece la pena bajar a 25x25 para debuggear

        // Llamo a solve y ya tengo el vector de p

        // Con las presiones resueltas, tengo que actualizar las velocidades
        // Primero meto el vector de p, en la variable pressure que es un campo 2d de los valores
        // El array tiene un getData, que es un puntero y podemos recorrerlo de i a 10.000 para actualizar todos los
        // valores

        // Parte final es diapositiva 19
        // Por cada velocidad tanto en x como en y, las actualizo usando la formula de la pag 19
        // Cuidado con las condiciones de contorno. Clampeo los indices, -1 pasa a 0 o 101 a 100
        // nueva velocidad = velocidad antigua - dt / densidad * Pi+1,j - Pi,j / dx

        
        // Ponemos a 0 las velocidades de los bordes
        for (int i = 0; i < velocityX.getSize().x; i++) {
            velocityX[Index2(i, 0)] = 0;
            velocityX[Index2(i, velocityX.getSize().y - 1)] = 0;
        }
        for (int i = 0; i < velocityY.getSize().x; i++) {
            velocityY[Index2(i, 0)] = 0;
            velocityY[Index2(i, velocityY.getSize().y - 1)] = 0;
        }
        for (int i = 0; i < velocityX.getSize().y; i++) {
            velocityX[Index2(0, i)] = 0;
            velocityX[Index2(velocityX.getSize().x - 1, i)] = 0;
        }
        for (int i = 0; i < velocityY.getSize().y; i++) {
            velocityY[Index2(0, i)] = 0;
            velocityY[Index2(velocityY.getSize().x - 1, i)] = 0;
        }

        auto& size = grid.getSize();
        auto& dx = grid.getCellDx();

        // Calculamos el vector de derecho
        std::vector<double> b(size.x * size.y);
        std::vector<double> p(size.x * size.y);
        SparseMatrix<double> A(size.x * size.y);

        double c = -Scene::kDensity / dt;
        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                Index2 index(i, j);
                auto u = velocityX[index];
                auto rightU = value_or_zero(Index2(index.x + 1, index.y), velocityX);
                auto dU = (rightU - u) / dx.x;

                auto v = velocityY[index];
                auto upV = value_or_zero(Index2(index.x, index.y + 1), velocityY);
                auto dV = (upV - v) / dx.y;

                auto linIndex = (j * size.x) + i;
                b[linIndex] = c * (dU + dV);
            }
        }

        // Calculamos la matriz A
        auto invDx = 1 / (dx.x * dx.x);
        auto invDy = 1 / (dx.y * dx.y);

        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                auto linIndex = (j * size.x) + i;
                Index2 index(i, j);

                float elementX = 2.0f * invDx;                
                float elementY = 2.0f * invDy;                
                if (i == 0 || i == size.x - 1)
                    elementX = invDx;
                if (j == 0)
                    elementY = invDy;                

                int linearID = pressure.getLinearIndex(i, j);
                Index2 point_ij(i, j);
                A.set_element(linearID, linearID, elementX + elementY);

                if (j == size.y - 1)
                    A.set_element(linearID, linearID, elementX + elementY);

                Index2 P_left(i - 1, j);
                if (P_left.x < size.x)
                    A.set_element(
                        linearID, pressure.getLinearIndex(P_left.x, P_left.y), -invDx);
                Index2 P_right(i + 1, j);
                if (P_right.x < size.x)
                    A.set_element(
                        linearID, pressure.getLinearIndex(P_right.x, P_right.y), -invDx);
                Index2 P_down(i, j - 1);
                if (P_down.y < size.y)
                    A.set_element(
                        linearID, pressure.getLinearIndex(P_down.x, P_down.y), -invDy);
                Index2 P_up(i, j + 1);
                if (P_up.y < size.y)
                    A.set_element(linearID, pressure.getLinearIndex(P_up.x, P_up.y), -invDy);
            }
        }


        // Resolvemos el sistema
        PCGSolver<double> solver;
        double residual_out;        
        int iterations;
        solver.solve(A, b, p, residual_out, iterations);

        // Actualizamos las presiones
        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                Index2 index(i, j);
                auto linIndex = (j * size.x) + i;
                pressure[index] = p[linIndex];
            }
        }

        // Calculamos las nuevas velocidades
        float density = Scene::kDensity;
        Array2<float> velX(velocityX.getSize());
        Array2<float> velY(velocityY.getSize());
        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                Index2 p(i, j);

                float ro = check(p, pressure);

                Index2 horizontal = Index2(p.x + 1, p.y);
                float horizontalV = check(horizontal, velocityX);
                float horizontalP = check(horizontal, pressure);
                float calcX = horizontalV - (dt / density) * (horizontalP - ro) / dx.x;

                Index2 szX = velX.getSize();
                if (horizontal.x >= 0 && horizontal.x < szX.x && horizontal.y >= 0 && horizontal.y < szX.y)
                    velX[horizontal] = calcX;

                Index2 vertical = Index2(p.x, p.y + 1);
                float verticalV = check(vertical, velocityY);
                float verticalP = check(vertical, pressure);
                float calcY = verticalV - (dt / density) * (verticalP - ro) / dx.y;

                Index2 szY = velY.getSize();
                if (vertical.x >= 0 && vertical.x < szY.x && vertical.y >= 0 && vertical.y < szY.y)
                    velY[vertical] = calcY;
            }
        }
        velocityX = velX;
        velocityY = velY;        
    }
}

}  // namespace asa
