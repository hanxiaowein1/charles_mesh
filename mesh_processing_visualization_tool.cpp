#include "mesh_processing_visualization_tool.h"
#include <QFileDialog>
#include <QMessageBox>
#include <format>
#include "viewer_middle_layer.h"
#include "mesh_factory.h"
#include "charles_mesh.h"

MeshProcessingVisualizationTool::MeshProcessingVisualizationTool(QWidget* parent) : QMainWindow(parent)
{
    setWindowTitle("mesh processing");

    // Create central widget and layout
    this->mainWidget = new QWidget(this);
    setCentralWidget(this->mainWidget);

    this->mainLayout = new QVBoxLayout(this->mainWidget);
    this->meshViewLayout = new QHBoxLayout();
    this->meshProcessingLayout = new QHBoxLayout();

    // mesh viewing layout
    this->chooseMeshButton = new QPushButton("choose mesh", this->mainWidget);

    this->meshPathLabel = new QLabel("", this->mainWidget);
    this->meshPathLabel->setFrameStyle(QFrame::Box | QFrame::Sunken);
    this->meshPathLabel->setLineWidth(2);
    this->meshPathLabel->setMidLineWidth(0);
    this->meshPathLabel->setStyleSheet(
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
    this->meshPathLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    this->viewMeshButton = new QPushButton("view", this->mainWidget);
    this->viewMeshButton->setEnabled(false);

    this->meshViewLayout->addWidget(this->chooseMeshButton);
    this->meshViewLayout->addWidget(this->meshPathLabel);
    this->meshViewLayout->addWidget(this->viewMeshButton);

    this->mainLayout->addLayout(this->meshViewLayout);


    // mesh processing layout
    this->descriptionLabel = new QLabel(this->mainWidget);
    this->descriptionLabel->setText("action");
    QFont desc_font = this->descriptionLabel->font();
    desc_font.setPointSize(12);
    desc_font.setBold(true);
    this->descriptionLabel->setFont(desc_font);
    QPalette desc_palette = this->descriptionLabel->palette();
    desc_palette.setColor(QPalette::WindowText, QColor(0x8B, 0x00, 0x00));
    this->descriptionLabel->setPalette(desc_palette);
    this->descriptionLabel->setFrameStyle(QFrame::NoFrame);
    this->descriptionLabel->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);
    this->descriptionLabel->setWordWrap(true);

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

    connect(chooseMeshButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onChooseMeshClicked);
    connect(viewMeshButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onViewMeshClicked);
    connect(simplifyButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onSimplifyClicked);
    connect(previewButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::onPreviewClicked);
    connect(saveMeshButton, &QPushButton::clicked, this, &MeshProcessingVisualizationTool::saveMeshClicked);
}

void MeshProcessingVisualizationTool::onChooseMeshClicked()
{
    // clear state
    this->viewMeshButton->setEnabled(false);
    this->simplifyButton->setEnabled(false);
    this->previewButton->setEnabled(false);
    this->saveMeshButton->setEnabled(false);
    this->processing_mesh = nullptr;

    QString meshPath = QFileDialog::getOpenFileName(
        this,
        "choose mesh",
        QDir::homePath(),
        "All Files (*.obj)"
    );

    if(!meshPath.isEmpty())
    {
        this->meshPathLabel->setText(meshPath);
        QFontMetrics metrics(this->meshPathLabel->font());
        QString elidedText = metrics.elidedText(meshPath, Qt::ElideMiddle, this->meshPathLabel->width() - 16);
        this->meshPathLabel->setText(elidedText);
        this->meshPathLabel->setToolTip(meshPath);

        this->viewMeshButton->setEnabled(true);
        this->simplifyButton->setEnabled(true);
    }
    else
    {
        // set mesh path label to empty
        this->meshPathLabel->setText("");
    }
}

void MeshProcessingVisualizationTool::onViewMeshClicked()
{
    std::string meshPath = this->get_mesh_path_from_mesh_path_label();
    if(meshPath == "")
    {
        this->viewMeshButton->setEnabled(false);
        return;
    }
    charles_mesh::mesh_viewer(meshPath);
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

    this->processing_mesh->edge_collapse(10);
    std::cout << std::format("simplify mesh {} success", meshPath) << std::endl;

    this->previewButton->setEnabled(true);
    this->saveMeshButton->setEnabled(true);
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
        this->processing_mesh->save_obj(savePath.toStdString());
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