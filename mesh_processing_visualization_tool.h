// the simple template of choose file and process is not enough for current use, this code is created for visulization tool for easily processing mesh file
#ifndef __CHARLES_MESH_PROCESSING_VISUALIZATION_TOOL__
#define __CHARLES_MESH_PROCESSING_VISUALIZATION_TOOL__

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "charles_mesh.h"

class MeshProcessingVisualizationTool : public QMainWindow
{
    Q_OBJECT

public:
    MeshProcessingVisualizationTool(QWidget* parent = nullptr);
    ~MeshProcessingVisualizationTool();

private slots:
    void onChooseMeshClicked();
    void onViewMeshClicked();
    void onSimplifyClicked();
    void onPreviewClicked();
    void saveMeshClicked();
private:
    QWidget *mainWidget;

    QVBoxLayout* mainLayout;

    QHBoxLayout* meshViewLayout;
    QPushButton *chooseMeshButton;
    QLabel* meshPathLabel;
    QPushButton* viewMeshButton;

    QHBoxLayout* meshProcessingLayout;
    QLabel* descriptionLabel;
    QPushButton* simplifyButton;
    QPushButton* previewButton;
    QPushButton* saveMeshButton;

private: 
    std::shared_ptr<charles_mesh::Mesh<charles_mesh::Point3D>> processing_mesh;

private:
    std::string get_mesh_path_from_mesh_path_label();

};



#endif