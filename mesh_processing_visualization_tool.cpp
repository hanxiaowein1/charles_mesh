#include "mesh_processing_visualization_tool.h"
#include <QFileDialog>
#include <QMessageBox>
#include <format>
#include <QThread>
#include "viewer_middle_layer.h"
#include "mesh_factory.h"
#include "charles_mesh.h"
#include "marching_cubes_33/mc33.h"

MeshProcessingVisualizationTool::MeshProcessingVisualizationTool(QWidget* parent) : QMainWindow(parent)
{
    setWindowTitle("mesh processing");

    // Create central widget and layout
    this->mainWidget = new QWidget(this);
    setCentralWidget(this->mainWidget);

    this->mainLayout = new QVBoxLayout(this->mainWidget);
    this->mc33ViewLayout = new QHBoxLayout();
    this->meshViewLayout = new QHBoxLayout();
    this->meshProcessingLayout = new QHBoxLayout();


    // mc33 view layout
    this->chooseSdButton = new QPushButton("choose signed distance file", this->mainWidget);

    this->sdPathLabel = new QLabel("", this->mainWidget);
    this->set_label_style_template(this->sdPathLabel);

    this->generateMeshButton = new QPushButton("generate", this->mainWidget);
    this->generateMeshButton->setEnabled(false);

    this->mc33ViewLayout->addWidget(this->chooseSdButton);
    this->mc33ViewLayout->addWidget(this->sdPathLabel);
    this->mc33ViewLayout->addWidget(this->generateMeshButton);
    this->mainLayout->addLayout(this->mc33ViewLayout);

    // mesh viewing layout
    this->chooseMeshButton = new QPushButton("choose mesh", this->mainWidget);

    this->meshPathLabel = new QLabel("", this->mainWidget);
    this->set_label_style_template(this->meshPathLabel);

    this->viewMeshButton = new QPushButton("view", this->mainWidget);
    this->viewMeshButton->setEnabled(false);

    this->meshViewLayout->addWidget(this->chooseMeshButton);
    this->meshViewLayout->addWidget(this->meshPathLabel);
    this->meshViewLayout->addWidget(this->viewMeshButton);

    this->mainLayout->addLayout(this->meshViewLayout);


    // mesh processing layout
    this->descriptionLabel = new QLabel(this->mainWidget);
    this->descriptionLabel->setText("no mesh");
    QFont desc_font = this->descriptionLabel->font();
    desc_font.setPointSize(12);
    desc_font.setBold(true);
    this->descriptionLabel->setFont(desc_font);
    QPalette desc_palette = this->descriptionLabel->palette();
    desc_palette.setColor(QPalette::WindowText, QColor(0x8B, 0x00, 0x00));
    this->descriptionLabel->setPalette(desc_palette);
    this->descriptionLabel->setFrameStyle(QFrame::NoFrame);
    this->descriptionLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);
    this->descriptionLabel->setWordWrap(false);

    this->simplifyButton = new QPushButton("simplify", this->mainWidget);
    this->simplifyButton->setEnabled(false);

    this->previewButton = new QPushButton("preview", this->mainWidget);
    this->previewButton->setEnabled(false);

    this->saveMeshButton = new QPushButton("save", this->mainWidget);
    this->saveMeshButton->setEnabled(false);

    this->meshProcessingLayout->addWidget(this->descriptionLabel);
    this->meshProcessingLayout->addWidget(this->simplifyButton);
    this->meshProcessingLayout->addWidget(this->previewButton);
    this->meshProcessingLayout->addWidget(this->saveMeshButton);
    this->mainLayout->addLayout(this->meshProcessingLayout);

    this->mainLayout->addStretch();
    this->mainLayout->setContentsMargins(10, 10, 10, 10);

    connect(this->chooseSdButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onChooseSdClicked);
    connect(this->generateMeshButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onGenerateMeshClicked);
    connect(this->chooseMeshButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onChooseMeshClicked);
    connect(this->viewMeshButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onViewMeshClicked);
    connect(this->simplifyButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onSimplifyClicked);
    connect(this->previewButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onPreviewClicked);
    connect(this->saveMeshButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::saveMeshClicked);
}

void MeshProcessingVisualizationTool::set_label_style_template(QLabel* qLabel)
{
    qLabel->setFrameStyle(QFrame::Box | QFrame::Sunken);
    qLabel->setLineWidth(2);
    qLabel->setMidLineWidth(0);
    qLabel->setStyleSheet(
        "QLabel {"
        "    background-color: white;"
        "    border: 2px solid #cccccc;"
        "    border-radius: 5px;"
        "    padding: 8px;"
        "    min-width: 250px;" // Adjusted width
        "    min-height: 24px;"
        "    font-size: 12px;"
        "}"
    );
    qLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
}

bool MeshProcessingVisualizationTool::choose_file_template(QLabel* qLabel, const QString& description, const QString& filter)
{
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "choose signed distance file",
        QDir::homePath(),
        filter
    );
    if(!filePath.isEmpty())
    {
        qLabel->setText(filePath);
        QFontMetrics metrics(qLabel->font());
        QString elidedText = metrics.elidedText(filePath, Qt::ElideMiddle, qLabel->width() - 16);
        qLabel->setText(elidedText);
        qLabel->setToolTip(filePath);
        return true;
    }
    return false;
}

void MeshProcessingVisualizationTool::pop_up_modal_window(std::function<void()> task, const QString& description)
{
    QDialog *dialog = new QDialog(this);
    dialog->setWindowTitle("Processing...");
    dialog->setModal(true);
    dialog->setWindowFlags(dialog->windowFlags() & ~Qt::WindowCloseButtonHint);

    QVBoxLayout *layout = new QVBoxLayout(dialog);
    layout->addWidget(new QLabel(description));
    dialog->setLayout(layout);

    QThread* thread = new QThread;
    Worker *worker = new Worker(task, nullptr);
    worker->moveToThread(thread);

    connect(thread, &QThread::started, worker, &Worker::doWork);
    connect(worker, &Worker::workFinished, dialog, &QDialog::accept);
    connect(worker, &Worker::workFinished, thread, &QThread::quit);
    connect(worker, &Worker::workFinished, worker, &QObject::deleteLater);
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);

    thread->start();
    dialog->exec();
    delete dialog;
}


void MeshProcessingVisualizationTool::onGenerateMeshClicked()
{
    auto sdPath = this->get_text_from_label(this->sdPathLabel);
    if(sdPath == "")
    {
        this->generateMeshButton->setEnabled(false);
        return;
    }
    else
    {
        // start to processing generating mesh
        auto task = [sdPath, this](){
            auto [vertice, triangles] = generate_mesh(sdPath);
            // convert vertice to Point3D
            std::vector<charles_mesh::Point3D> _vertice;
            for(const auto& vertex: vertice)
            {
                charles_mesh::Point3D point;
                point.x = vertex[0];
                point.y = vertex[1];
                point.z = vertex[2];
                _vertice.emplace_back(point);
            }
            this->processing_mesh = std::make_shared<charles_mesh::Mesh<charles_mesh::Point3D>>(_vertice, triangles);
        };
        this->pop_up_modal_window(task, "please wait, mesh generating in progress...");
        this->viewMeshButton->setEnabled(true);
        this->simplifyButton->setEnabled(true);
        this->previewButton->setEnabled(true);
        this->saveMeshButton->setEnabled(true);
        this->descriptionLabel->setText(
            std::format("vertex: {}, triangle: {}", this->processing_mesh->vertices.size(), this->processing_mesh->faces.size()).c_str()
        );
    }
}

void MeshProcessingVisualizationTool::onChooseSdClicked()
{
    bool ret = this->choose_file_template(this->sdPathLabel, "choose signed distance file", "All Files (*.pb)");
    if(ret)
    {
        this->generateMeshButton->setEnabled(true);
    }
}

void MeshProcessingVisualizationTool::onChooseMeshClicked()
{
    // clear state
    this->viewMeshButton->setEnabled(false);
    this->simplifyButton->setEnabled(false);
    this->previewButton->setEnabled(false);
    this->saveMeshButton->setEnabled(false);
    this->processing_mesh = nullptr;

    bool ret = this->choose_file_template(this->meshPathLabel, "choose mesh file", "All Files (*.obj)");
    if(ret)
    {
        auto meshPath = this->get_text_from_label(this->meshPathLabel);
        charles_mesh::ObjMeshIO obj_mesh_io;
        this->processing_mesh = obj_mesh_io.load_mesh(meshPath);
        this->viewMeshButton->setEnabled(true);
        this->simplifyButton->setEnabled(true);
        this->previewButton->setEnabled(true);
        this->saveMeshButton->setEnabled(true);
        this->descriptionLabel->setText(
            std::format("vertex: {}, triangle: {}", this->processing_mesh->vertices.size(), this->processing_mesh->faces.size()).c_str()
        );
    }
    else
    {
        // set mesh path label to empty
        this->meshPathLabel->setText("");
    }
}

void MeshProcessingVisualizationTool::onViewMeshClicked()
{
    std::string meshPath = this->get_text_from_label(this->meshPathLabel);
    if(this->processing_mesh == nullptr)
    {
        QMessageBox::warning(this, "no processing mesh", "please select a mesh or generate it from signed distance file first");
        return;
    }
    else
    {
        charles_mesh::mesh_viewer(this->processing_mesh);
    }
}

std::string MeshProcessingVisualizationTool::get_text_from_label(QLabel* qLabel)
{
    QString text = qLabel->toolTip();
    if(text.isEmpty())
    {
        text = qLabel->text();
    }
    return text.toStdString();
}

std::string MeshProcessingVisualizationTool::get_mesh_path_from_mesh_path_label()
{
    QString meshPath = this->meshPathLabel->toolTip();
    if(meshPath.isEmpty())
    {
        meshPath = this->meshPathLabel->text();
    }
    if(meshPath.isEmpty())
    {
        QMessageBox::warning(this, "No File", "please select a mesh first");
        return "";
    }
    return meshPath.toStdString();
}


void MeshProcessingVisualizationTool::onSimplifyClicked()
{
    std::string meshPath = "";
    if(this->processing_mesh == nullptr)
    {
        std::string meshPath = this->get_mesh_path_from_mesh_path_label();
        if(meshPath == "")
        {
            this->simplifyButton->setEnabled(false);
            return;
        }
        charles_mesh::ObjMeshIO obj_mesh_io;
        this->processing_mesh = obj_mesh_io.load_mesh(meshPath);
        std::cout << std::format("load mesh {} success, start simplifying...", meshPath) << std::endl;
    }
    else
    {
        std::cout << std::format("processing mesh already exists! This step will use it to simplify!") << std::endl;
    }

    this->pop_up_modal_window([this](){
        this->processing_mesh->edge_collapse(10);
    }, "please wait, edge collapse in progress");
    this->descriptionLabel->setText(
        std::format("vertex: {}, triangle: {}", this->processing_mesh->vertices.size(), this->processing_mesh->faces.size()).c_str()
    );
    std::cout << std::format("simplify mesh {} success", meshPath) << std::endl;
}

void MeshProcessingVisualizationTool::saveMeshClicked()
{
    QString savePath = QFileDialog::getSaveFileName(
        this,
        "save file",
        QDir::homePath(),
        "All Files (*.obj)"
    );
    if(!savePath.isEmpty())
    {
        std::cout << std::format("save mesh to {}", savePath.toStdString()) << std::endl;
        this->pop_up_modal_window([this, savePath](){
            this->processing_mesh->save_obj(savePath.toStdString());
        }, "please wait, mesh saving in progress...");
        std::cout << "save complete!" << std::endl;
    }
}


void MeshProcessingVisualizationTool::onPreviewClicked()
{
    charles_mesh::mesh_viewer(this->processing_mesh);
}


MeshProcessingVisualizationTool::~MeshProcessingVisualizationTool()
{

}