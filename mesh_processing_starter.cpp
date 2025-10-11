#include "mesh_processing_visualization_tool.h"
#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    app.setStyle("Fusion");

    MeshProcessingVisualizationTool window;
    window.show();

    return app.exec();
}