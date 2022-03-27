#include "Scene.h"

#include "Numeric/PCGSolver.h"

namespace asa
{
namespace
{

template<class T>
T value_or_zero(const Index2 &index, const Array2<T> &data)
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

bool is_inbounds(const Index2 &a, const Index2 &b)
{
    return a.x < b.x - 1 && a.x >= 0 && a.y < b.y - 1 && a.y >= 0;
}

}  // namespace

// advection
void Fluid2::fluidAdvection(const float dt)
{
    auto minPos = grid.getDomain().minPosition;
    auto maxPos = grid.getDomain().maxPosition;
    {
        Array2<asa::Vector3> inkCopy(inkRGB);
        auto& size = grid.getSize();        

        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                // Convierto el i j a la posicion
                auto index = Index2(i, j);
                auto pos = grid.getCellPos(index);
                // pos es x, y del espacio

                auto u = (velocityX[index] + velocityX[Index2(clamp(index.x + 1, 0, size.x), index.y)]) / 2;
                auto v = (velocityY[index] + velocityY[Index2(index.x, clamp(index.y + 1, 0, size.y))]) / 2;

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
        auto size = grid.getSize();
        int halfW = 4;
        int startY = 5;
        int height = 20;

        for (int i = (size.x / 2) - halfW; i <= (size.x / 2) + halfW; i++) {        
            for (int j = startY; j <= height; j++) {            
                Index2 idx(i, j);

                if (i < size.x / 2) {
                    inkRGB[idx] = Vector3(1, 0, 1);                    
                } else if (i < (size.x / 2) + halfW) {
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
        // velocidad en y = velicidad en n * dt * -9.81
        
        for (int i = 0; i < grid.getSizeFacesY().x; i++) {
            for (int j = 0; j < grid.getSizeFacesY().y; j++) {
                auto index = Index2(i, j);
                velocityY[index] += dt * Scene::kGravity;  // OJO! La gravedad ya esta definida con -
            }
        }
    }
}

void Fluid2::fluidViscosity(const float dt)
{    
    // u* = u + (dt / density) * viscosity * (dif. finitas de velocidades)
    if (Scene::testcase >= Scene::SMOKE) {        
        auto uCopy(velocityX);
        auto vCopy(velocityY);
        auto &sizeU = velocityX.getSize();
        auto &sizeV = velocityY.getSize();
        auto dx = grid.getCellDx();
        auto c = (dt * Scene::kViscosity) / Scene::kDensity;

        // Calculamos la parte U del vector
        for (int i = 0; i < sizeU.x; i++) {
            for (int j = 0; j < sizeU.y; j++) {
                auto index = Index2(i, j);
                if (i == 0 || j == 0 || i == sizeU.x - 1 || j == sizeU.y - 1) {
                    velocityX[index] = 0;
                    continue;
                }
                
                auto u = uCopy[index];

                auto rightU = uCopy[Index2(i + 1, j)];                
                auto leftU = uCopy[Index2(i - 1, j)];                
                auto upU = uCopy[Index2(i, j + 1)];                
                auto downU = uCopy[Index2(i, j - 1)];                

                auto difU1 = (rightU - 2.0f * u + leftU) / (dx.x * dx.x);
                auto difU2 = (upU - 2.0f * u + downU) / (dx.y * dx.y);
                auto difU = difU1 + difU2;
                
                velocityX[index] = u + (c * difU);
            }
        }

        // Calculamos la parte V del vector
        for (int i = 0; i < sizeV.x; i++) {
            for (int j = 0; j < sizeV.y; j++) {
                auto index = Index2(i, j);
                if (i == 0 || j == 0 || i == sizeV.x - 1 || j == sizeV.y - 1) {
                    velocityY[index] = 0;
                    continue;
                }
                auto v = vCopy[index];

                auto rightV = vCopy[Index2(i + 1, j)];                
                auto leftV = vCopy[Index2(i - 1, j)];                
                auto upV = vCopy[Index2(i, j + 1)];                
                auto downV = vCopy[Index2(i, j - 1)];

                auto difV1 = (rightV - 2.0f * v + leftV) / (dx.x * dx.x);
                auto difV2 = (upV - 2.0f * v + downV) / (dx.y * dx.y);
                auto difV = difV1 + difV2;
                
                velocityY[index] = v + (c * difV);
            }
        }            
    }
}

void Fluid2::fluidPressureProjection(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {        
        // Ponemos a 0 las velocidades de los bordes simulando paredes
        for (int i = 0; i < velocityX.getSize().x; i++) {
            velocityX[Index2(i, 0)] = 0;
            velocityX[Index2(i, velocityX.getSize().y - 1)] = 0;
        }
        for (int i = 0; i < velocityY.getSize().x; i++) {
            velocityY[Index2(i, 0)] = 0;
            velocityY[Index2(i, velocityY.getSize().y - 1)] = 0;
        }
        for (int j = 0; j < velocityX.getSize().y; j++) {
            velocityX[Index2(0, j)] = 0;
            velocityX[Index2(velocityX.getSize().x - 1, j)] = 0;
        }
        for (int j = 0; j < velocityY.getSize().y; j++) {
            velocityY[Index2(0, j)] = 0;
            velocityY[Index2(velocityY.getSize().x - 1, j)] = 0;
        }

        auto& size = grid.getSize();
        auto& dx = grid.getCellDx();

        // Sistema de ec. lineales a resolver: A * p = b       
        std::vector<double> b(size.x * size.y);
        std::vector<double> p(size.x * size.y);
        SparseMatrix<double> A(size.x * size.y);

        // Calculamos el vector de derecho        
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
                auto linIndex = pressure.getLinearIndex(i, j);
                Index2 index(i, j);            

                // ponemos la diagonal principal
                auto xVal = 2 * invDx;
                auto yVal = 2 * invDy;
                // En las esquinas, como hay paredes por los lados
                // La derivada es 0 y el valor es diferente
                if (i == 0 && j == 0) {
                    xVal = invDx;
                }
                if (i == size.x - 1 && j == size.y - 1) {
                    yVal = invDy;
                }                
                A.set_element(linIndex, linIndex, xVal + yVal);

                // A la izquierda
                if (i > 0) {
                    auto leftIndex = pressure.getLinearIndex(i - 1, j);
                    A.set_element(linIndex, leftIndex, -invDx);
                }
                // A la derecha
                if (i < size.x - 1) {
                    auto rightIndex = pressure.getLinearIndex(i + 1, j);
                    A.set_element(linIndex, rightIndex, -invDx);
                }

                // Arriba
                if (j < size.y - 1) {
                    auto upIndex = pressure.getLinearIndex(i, j + 1);
                    A.set_element(linIndex, upIndex, -invDy);
                }
                // Abajo
                if (j > 0) {
                    auto downIndex = pressure.getLinearIndex(i, j - 1);
                    A.set_element(linIndex, downIndex, -invDy);
                }                                                                                          
            }
        }


        // Resolvemos el sistema
        PCGSolver<double> solver;
        double residual_out;        
        int iterations;
        solver.solve(A, b, p, residual_out, iterations);

        // Con la solucion, actualizamos las presiones
        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                Index2 index(i, j);
                auto linIndex = (j * size.x) + i;
                pressure[index] = p[linIndex];
            }
        }

        // Calculamos las nuevas velocidades        
        Array2<float> uCopy(velocityX);
        Array2<float> vCopy(velocityY);    
        double cV = dt / Scene::kDensity;
        for (int i = 0; i < size.x; i++) {
            for (int j = 0; j < size.y; j++) {
                Index2 index(i, j);
                Index2 indexRight(i + 1, j);
                Index2 indexUp(i, j + 1);

                auto p = value_or_zero(index, pressure);
                auto rightP = value_or_zero(indexRight, pressure);
                auto upP = value_or_zero(indexUp, pressure);
                auto u = value_or_zero(indexRight, uCopy);
                if (is_inbounds(indexRight, velocityX.getSize())) {
                    velocityX[indexRight] = u - (cV * ((rightP - p) / dx.x));
                }

                auto v = value_or_zero(indexUp, vCopy);
                if (is_inbounds(indexUp, velocityY.getSize())) {
                    velocityY[indexUp] = v - (cV * ((upP - p) / dx.y));
                }                
            }
        }     
    }
}

}  // namespace asa
