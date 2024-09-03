#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtkAbstractPicker.h>
#include <vtkBuffer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>
#include <vtkVertexGlyphFilter.h>

constexpr auto* path {
    "/home/creeper/workspace/sentry"
    "/ignore/develop_ws/pcd/1716207472.pcd"
};

auto pointsSource = vtkNew<vtkPoints> {};
auto cloudPolyData = vtkNew<vtkPolyData> {};
auto cloudMapper = vtkNew<vtkPolyDataMapper> {};
auto cloudActor = vtkNew<vtkActor> {};

auto pointsClicked = vtkNew<vtkPoints> {};
auto clickedPolyData = vtkNew<vtkPolyData> {};
auto clickedMapper = vtkNew<vtkPolyDataMapper> {};
auto clickedActor = vtkNew<vtkActor> {};

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
auto pointCloud = std::make_shared<::PointCloudT>();
auto vertexFilter = vtkNew<vtkVertexGlyphFilter> {};

class PickStyle : public vtkInteractorStyleTrackballCamera {
public:
    static PickStyle* New();
    vtkTypeMacro(PickStyle, vtkInteractorStyleTrackballCamera);

    void OnLeftButtonDown() override {

        this->Interactor->GetPicker()->Pick(
            this->Interactor->GetEventPosition()[0],
            this->Interactor->GetEventPosition()[1],
            0,
            this->Interactor->GetRenderWindow()
                ->GetRenderers()
                ->GetFirstRenderer());

        double point[3];
        this->Interactor->GetPicker()->GetPickPosition(point);

        std::printf("Picked: %.3f, %.3f, %.3f\n",
            point[0], point[1], point[2]);

        pointsClicked->InsertNextPoint(point[0], point[1], point[2]);

        vertexFilter->SetInputData(clickedPolyData);
        vertexFilter->Update();
        clickedPolyData->ShallowCopy(vertexFilter->GetOutput());

        clickedPolyData->GetPoints()->Modified();
        // pointsClicked->Modified();

        // Forward events.
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }
};
vtkStandardNewMacro(PickStyle);

auto main() -> int {
    /// Load Point Source
    if (pcl::io::loadPCDFile(::path, *pointCloud) == -1) {
        std::cout << "load failed" << '\n';
    }

    /// Make point data sheet
    for (const auto point : *pointCloud) {
        pointsSource->InsertNextPoint(point.x, point.y, point.z);
    }
    cloudPolyData->SetPoints(pointsSource);

    clickedPolyData->SetPoints(pointsClicked);

    /// Make vertex data
    vertexFilter->SetInputData(cloudPolyData);
    vertexFilter->Update();
    cloudPolyData->ShallowCopy(vertexFilter->GetOutput());

    cloudMapper->SetInputData(cloudPolyData);
    cloudActor->SetMapper(cloudMapper);

    clickedMapper->SetInputData(clickedPolyData);
    clickedActor->SetMapper(clickedMapper);

    /// Configuration
    cloudActor->GetProperty()->SetPointSize(5);
    cloudActor->GetProperty()->SetColor(1, 1, 1);

    clickedActor->GetProperty()->SetPointSize(10);
    clickedActor->GetProperty()->SetColor(1, 0, 0);

    // Render
    auto renderer = vtkNew<vtkRenderer> {};
    auto renderWindow = vtkNew<vtkRenderWindow> {};

    renderWindow->SetWindowName("Point");
    renderWindow->AddRenderer(renderer);

    vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    renderWindowInteractor->SetRenderWindow(renderWindow);

    vtkNew<PickStyle> style;
    renderWindowInteractor->SetInteractorStyle(style);

    renderWindow->SetSize(1440, 720);

    renderer->AddActor(cloudActor);
    renderer->AddActor(clickedActor);
    renderer->SetBackground(0, 0, 0);

    renderWindow->Render();
    renderWindowInteractor->Start();
}
