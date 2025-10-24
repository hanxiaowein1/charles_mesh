// the simple template of choose file and process is not enough for current use, this code is created for visulization tool for easily processing mesh file
#ifndef __CHARLES_MESH_PROCESSING_VISUALIZATION_TOOL__
#define __CHARLES_MESH_PROCESSING_VISUALIZATION_TOOL__

#include <thread>
#include <chrono>
#include <functional>

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "charles_mesh.h"

class Worker : public QObject
{
    Q_OBJECT
public slots:
    void doWork()
    {
        // std::this_thread::sleep_for(std::chrono::seconds(3));
        this->m_job();
        emit workFinished();
    }
signals:
    void workFinished();
private:
    std::function<void()> m_job;
public:
    Worker(std::function<void()> job, QObject* parent = nullptr) : QObject(parent), m_job(job){}
    Worker(QObject* parent = nullptr) : QObject(parent){}
};

class MeshProcessingVisualizationTool : public QMainWindow
{
    Q_OBJECT

public:
    MeshProcessingVisualizationTool(QWidget* parent = nullptr);
    ~MeshProcessingVisualizationTool();

private slots:
    void onChooseSdClicked();
    void onGenerateMeshClicked();
    void onChooseMeshClicked();
    void onViewMeshClicked();
    void onSimplifyClicked();
    void onPreviewClicked();
    void saveMeshClicked();
private:
    QWidget *mainWidget;

    QVBoxLayout* mainLayout;

    QHBoxLayout* mc33ViewLayout;
    // choose signed distance file
    QPushButton* chooseSdButton;
    QLabel* sdPathLabel;
    QPushButton* generateMeshButton;

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
    bool choose_file_template(QLabel* qLabel, const QString& description, const QString& filter);
    std::string get_text_from_label(QLabel* qLabel);
    void set_label_style_template(QLabel* qLabel);
    void pop_up_modal_window(std::function<void()> task, const QString& description);
};



#endif