# Starter_Simulaciones_Robotica

Este repositorio es una base para empezar a desarrollar simulaciones de robótica con OMPL, PQP y OpenGL desde VS Code usando WSL o Linux.

## Requisitos

Antes de usar el proyecto, asegúrate de tener instalado lo siguiente en WSL o Linux:

- `g++` con soporte para C++17.
- OpenGL y sus dependencias de desarrollo.
- `glfw`.
- `assimp`.
- `Eigen3`.
- VS Code con acceso al entorno de WSL o Linux.

## Cómo empezar

1. Clona este repositorio en tu máquina.
2. Abre la carpeta raíz del proyecto en VS Code.
3. Asegúrate de estar trabajando en WSL o Linux.
4. Abre el archivo que quieres ejecutar, por ejemplo `pqpVisual.cpp` o `testPQP.cpp`.
5. Ejecuta la task llamada `WSL: Compile & Run current file`.

La task compila el archivo activo y genera el ejecutable dentro de `build/`.

## Cómo funciona la task

La task usa el archivo abierto en el editor como entrada, así que no necesitas cambiar nada del proyecto para probar otro ejemplo. Solo abre el `.cpp` que quieras correr y vuelve a lanzar la task.

Actualmente la task ya incluye las rutas y librerías que usa este repositorio:

- Includes de `ompl/` y `pqp/include`.
- La librería local de PQP ubicada en `pqp/lib`.
- Librerías del sistema como OpenGL, `glfw`, `assimp` y OMPL.

## Si falta alguna librería

Si agregas un archivo nuevo que necesite otra dependencia, solo extiende la task en `.vscode/tasks.json` agregando el include o el `-l` que corresponda.

Ejemplo:

```json
"args": [
	"-lc",
	"g++ ... -I/ruta/adicional -lnueva_libreria ..."
]
```

## Estructura relevante

- `.vscode/tasks.json`: task para compilar y ejecutar el archivo activo.
- `pqp/`: código y librería local de PQP.
- `ompl/`: código fuente de OMPL incluido en el repositorio.
- `models/`: modelos usados por los ejemplos.

## Nota

Este proyecto no depende de un `CMakeLists.txt` en la raíz. El flujo principal es abrir el archivo que quieras ejecutar y correr la task de VS Code.
