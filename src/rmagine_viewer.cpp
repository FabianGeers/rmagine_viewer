
#include <array>

// #define WITH_OPTIX
//embree as default
#if !defined(WITH_OPTIX) && !defined(WITH_VULKAN) && !defined(WITH_EMBREE)
  #define WITH_EMBREE
#endif


#if defined(WITH_EMBREE)
  #include <rmagine/simulation/SphereSimulatorEmbree.hpp>
  #include <rmagine/simulation/PinholeSimulatorEmbree.hpp>
  #include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
  #include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
  #include <rmagine/map/EmbreeMap.hpp>
  #include <rmagine/map/embree/embree_shapes.h>
#elif defined(WITH_OPTIX)
  #include <rmagine/simulation/SphereSimulatorOptix.hpp>
  #include <rmagine/simulation/PinholeSimulatorOptix.hpp>
  #include <rmagine/simulation/O1DnSimulatorOptix.hpp>
  #include <rmagine/simulation/OnDnSimulatorOptix.hpp>
  #include <rmagine/map/OptixMap.hpp>
  #include <rmagine/map/optix/optix_shapes.h>
#elif defined(WITH_VULKAN)
  #include <rmagine/simulation/SphereSimulatorVulkan.hpp>
  #include <rmagine/simulation/PinholeSimulatorVulkan.hpp>
  #include <rmagine/simulation/O1DnSimulatorVulkan.hpp>
  #include <rmagine/simulation/OnDnSimulatorVulkan.hpp>
  #include <rmagine/map/VulkanMap.hpp>
  #include <rmagine/map/vulkan/vulkan_shapes.hpp>
#else
  #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
#endif

#include <rmagine/types/sensors.h>
#include <rmagine/types/mesh_types.h>
#include <rmagine/math/linalg.h>

#include <rmagine/types/ouster_sensors.h>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#include "portable-file-dialogs.h"


namespace rm = rmagine;

#if defined(WITH_EMBREE)
  rm::EmbreeMapPtr generate_default_map()
  {
    rm::EmbreeScenePtr scene = std::make_shared<rm::EmbreeScene>();
    rm::EmbreeMeshPtr object = std::make_shared<rm::EmbreeCube>(1.0);

    { // positioning
      rm::Transform T = rm::Transform::Identity();
      T.t = {0.0, 0.0, 0.0};
      object->setTransform(T);
      rm::Vector3 scale = {1.0, 1.0, 1.0};
      object->setScale(scale);
      object->apply(); // apply transform related changes
    }
    object->name = "Cube";
    object->commit(); // commit changes to cube
    scene->add(object); // add cube to scene
    scene->commit(); // commit scene

    return std::make_shared<rm::EmbreeMap>(scene);
  }
#elif defined(WITH_OPTIX)
  rm::OptixMapPtr generate_default_map()
  {
    rm::OptixScenePtr scene = std::make_shared<rm::OptixScene>();
    rm::OptixMeshPtr object = std::make_shared<rm::OptixCube>(1.0);

    { // positioning
      rm::Transform T = rm::Transform::Identity();
      T.t = {0.0, 0.0, 0.0};
      object->setTransform(T);
      rm::Vector3 scale = {1.0, 1.0, 1.0};
      object->setScale(scale);
      object->apply(); // apply transform related changes
    }
    object->name = "Cube";
    object->commit(); // commit changes to cube
    scene->add(object); // add cube to scene
    scene->commit(); // commit scene

    return std::make_shared<rm::OptixMap>(scene);
  }
#elif defined(WITH_VULKAN)
  rm::VulkanMapPtr generate_default_map()
  {
    rm::VulkanScenePtr scene = std::make_shared<rm::VulkanScene>();
    rm::VulkanMeshPtr object = std::make_shared<rm::VulkanCube>(1.0);

    { // positioning
      rm::Transform T = rm::Transform::Identity();
      T.t = {0.0, 0.0, 0.0};
      object->setTransform(T);
      rm::Vector3 scale = {1.0, 1.0, 1.0};
      object->setScale(scale);
      object->apply(); // apply transform related changes
    }
    object->name = "Cube";
    object->commit(); // commit changes to cube
    scene->add(object); // add cube to scene
    scene->commit(); // commit scene

    return std::make_shared<rm::VulkanMap>(scene);
  }
#else
  #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
#endif

rm::SphericalModel generate_default_spherical_model()
{
  return rm::vlp16_900();
}

rm::PinholeModel generate_default_pinhole_model()
{
  return rm::example_pinhole();
}

rm::O1DnModel generate_default_o1dn_model()
{
  // from file
  return rm::example_o1dn();
}

rm::OnDnModel generate_default_ondn_model()
{
  // OnDn can be used for an orthogonal projection model
  return rm::example_ondn();
}

glm::mat4x4 glm_from_rm(const rm::Matrix4x4& M)
{
  glm::mat4x4 ret;
  std::memcpy(&ret, &M, 4 * 4 * sizeof(float));
  return ret;
}

glm::mat4x4 glm_from_rm(const rm::Transform& T)
{
  rm::Matrix4x4 M = (rm::Matrix4x4)T;
  // M.set(T);
  return glm_from_rm(M);
}

rm::Matrix4x4 rm_from_glm(const glm::mat4x4& M)
{
  rm::Matrix4x4 ret;
  std::memcpy(&ret, &M, 4 * 4 * sizeof(float));
  return ret;
}

using PolyscopeScene = std::unordered_map<unsigned int, polyscope::Structure*>;

#if defined(WITH_EMBREE)
  PolyscopeScene polyscope_scene_from_rmagine(rm::EmbreeScenePtr rm_scene)
  {
    PolyscopeScene ret;

    for(auto [rm_id, rm_geom] : rm_scene->geometries())
    {
      // convert rm mesh to polyscope
      auto rm_mesh = std::dynamic_pointer_cast<rm::EmbreeMesh>(rm_geom);
      if(rm_mesh)
      {
        // mesh found! create new polyscope element
        std::string poly_name = rm_mesh->name;
        if(poly_name == "")
        {
          std::stringstream ss;
          ss << "mesh" << rm_id;
          poly_name = ss.str();
        }
        polyscope::SurfaceMesh* poly_mesh = polyscope::registerSurfaceMesh(poly_name, rm_mesh->vertices(), rm_mesh->faces());
        poly_mesh->setTransform(glm_from_rm(rm_mesh->matrix()));
        poly_mesh->setTransparency(0.8);
        ret[rm_id] = poly_mesh;
      }
      //TODO: convert other things here... (not implemented yet)
    }

    return ret;
  }
#elif defined(WITH_OPTIX)
  PolyscopeScene polyscope_scene_from_rmagine(rm::OptixScenePtr rm_scene)
  {
    PolyscopeScene ret;

    for(auto [rm_id, rm_geom] : rm_scene->geometries())
    {
      // convert rm mesh to polyscope
      auto rm_mesh = std::dynamic_pointer_cast<rm::OptixMesh>(rm_geom);
      if(rm_mesh)
      {
        // mesh found! create new polyscope element
        std::string poly_name = rm_mesh->name;
        if(poly_name == "")
        {
          std::stringstream ss;
          ss << "mesh" << rm_id;
          poly_name = ss.str();
        }
        rm::Memory<rm::Vector, rm::RAM> verticies(rm_mesh->vertices.size());
        verticies = rm_mesh->vertices;
        rm::Memory<rm::Face, rm::RAM> faces(rm_mesh->faces.size());
        faces = rm_mesh->faces;
        polyscope::SurfaceMesh* poly_mesh = polyscope::registerSurfaceMesh(poly_name, verticies, faces);
        poly_mesh->setTransform(glm_from_rm(rm_mesh->matrix()));
        poly_mesh->setTransparency(0.8);
        ret[rm_id] = poly_mesh;
      }
      //TODO: convert other things here... (not implemented yet)
    }

    return ret;
  }
#elif defined(WITH_VULKAN)
  PolyscopeScene polyscope_scene_from_rmagine(rm::VulkanScenePtr rm_scene)
  {
    //TODO: will be a bit more complicated for vulkan 
  }
#else
  #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
#endif

#if defined(WITH_EMBREE)
  void synchronize(const PolyscopeScene& poly_scene, rm::EmbreeScenePtr rm_scene)
  {
    for(auto [rm_id, poly_mesh] : poly_scene)
    {
      auto rm_mesh = rm_scene->getAs<rm::EmbreeMesh>(rm_id);
      rm::Matrix4x4 M = rm_from_glm(poly_mesh->getTransform());
      rm::Transform T;
      rm::Vector scale;
      rm::decompose(M, T, scale);
      rm_mesh->setTransform(T);
      rm_mesh->setScale(scale);
      rm_mesh->apply();
      rm_mesh->commit();
    }
    rm_scene->commit();
  }
#elif defined(WITH_OPTIX)
  void synchronize(const PolyscopeScene& poly_scene, rm::OptixScenePtr rm_scene)
  {
    //TODO: at best this only works for mesh scenes (amd not instance scenes)
    auto geoms = rm_scene->geometries();
    for(auto [rm_id, poly_mesh] : poly_scene)
    {
      auto rm_mesh = geoms.at(rm_id)->this_shared<rm::OptixMesh>();
      rm::Matrix4x4 M = rm_from_glm(poly_mesh->getTransform());
      rm::Transform T;
      rm::Vector scale;
      rm::decompose(M, T, scale);
      rm_mesh->setTransform(T);
      rm_mesh->setScale(scale);
      rm_mesh->apply();
      rm_mesh->commit();
    }
    rm_scene->commit();
  }
#elif defined(WITH_VULKAN)
  void synchronize(const PolyscopeScene& poly_scene, rm::VulkanScenePtr rm_scene)
  {
    //TODO
  }
#else
  #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
#endif

/**
 * Simple polyscope-based viewer that shows the basic simulation capabilities of the rmagine library
 * 
 * run:
 * ./rmagine_polyscope_viewer [mesh_file]
 */
int main(int argc, char** argv)
{
  // ROS compatiple coordinates
  polyscope::view::setUpDir(polyscope::UpDir::ZUp);
  polyscope::view::setFrontDir(polyscope::FrontDir::XFront);

  // set the ground location manually
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::Tile;
  polyscope::options::groundPlaneHeightMode = polyscope::GroundPlaneHeightMode::Manual;
  polyscope::options::groundPlaneEnabled = false;
  polyscope::options::groundPlaneHeight = 0.;
  // a few options
  polyscope::options::programName = "Rmagine Viewer";
  polyscope::options::verbosity = 0;
  polyscope::options::usePrefsFile = false;
  

  // Initialize polyscope
  polyscope::init();
  polyscope::options::automaticallyComputeSceneExtents = false;

  // scenes: rmagine (raycasting acceleration) and polyscope (rendering) 
  
  #if defined(WITH_EMBREE)
    rm::EmbreeMapPtr rm_map;
  #elif defined(WITH_OPTIX)
    rm::OptixMapPtr rm_map;
  #elif defined(WITH_VULKAN)
    rm::VulkanMapPtr rm_map;
  #else
    #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
  #endif
  PolyscopeScene poly_scene;

  // construct a simulator
  #if defined(WITH_EMBREE)
    rm::SphereSimulatorEmbree rm_spherical_sim;
    rm::PinholeSimulatorEmbree rm_pinhole_sim;
    rm::O1DnSimulatorEmbree rm_o1dn_sim;
    rm::OnDnSimulatorEmbree rm_ondn_sim;
  #elif defined(WITH_OPTIX)
    rm::SphereSimulatorOptix rm_spherical_sim;
    rm::PinholeSimulatorOptix rm_pinhole_sim;
    rm::O1DnSimulatorOptix rm_o1dn_sim;
    rm::OnDnSimulatorOptix rm_ondn_sim;

    rm::Transform Tsb = rm::Transform::Identity();
    rm_spherical_sim.setTsb(Tsb);
    rm_pinhole_sim.setTsb(Tsb);
    rm_o1dn_sim.setTsb(Tsb);
    rm_ondn_sim.setTsb(Tsb);
  #elif defined(WITH_VULKAN)
    rm::SphereSimulatorVulkan rm_spherical_sim;
    rm::PinholeSimulatorVulkan rm_pinhole_sim;
    rm::O1DnSimulatorVulkan rm_o1dn_sim;
    rm::OnDnSimulatorVulkan rm_ondn_sim;

    rm::Transform Tsb = rm::Transform::Identity();
    rm_spherical_sim.setTsb(Tsb);
    rm_pinhole_sim.setTsb(Tsb);
    rm_o1dn_sim.setTsb(Tsb);
    rm_ondn_sim.setTsb(Tsb);
  #else
    #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
  #endif
  
  if(argc < 2)
  {
    rm_map = generate_default_map();
  }
  else
  {
    #if defined(WITH_EMBREE)
      rm_map = rm::import_embree_map(argv[1]);
    #elif defined(WITH_OPTIX)
      rm_map = rm::import_optix_map(argv[1]);
    #elif defined(WITH_VULKAN)
      rm_map = rm::import_vulkan_map(argv[1]);
    #else
      #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
    #endif
  }
  
  rm_spherical_sim.setMap(rm_map);
  rm_pinhole_sim.setMap(rm_map);
  rm_o1dn_sim.setMap(rm_map);
  rm_ondn_sim.setMap(rm_map);

  #if defined(WITH_EMBREE)
    poly_scene = polyscope_scene_from_rmagine(rm_map->scene);
  #elif defined(WITH_OPTIX)
    poly_scene = polyscope_scene_from_rmagine(rm_map->scene());
  #elif defined(WITH_VULKAN)
    poly_scene = polyscope_scene_from_rmagine(rm_map->scene());
  #else
    #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
  #endif
  
  rm::SphericalModel spherical_model = generate_default_spherical_model();
  rm::PinholeModel pinhole_model = generate_default_pinhole_model();
  rm::O1DnModel    o1dn_model = generate_default_o1dn_model();
  rm::OnDnModel    ondn_model = generate_default_ondn_model();


  // Spherical model presets
  float fov = 30.0;
  int   scan_lines = spherical_model.getHeight();
  int line_points = spherical_model.getWidth();
  rm::Interval spherical_range = spherical_model.range;

  // Pinhole model presets
  float fx = pinhole_model.f[0];
  float fy = pinhole_model.f[1];
  int pinhole_width = pinhole_model.getWidth();
  int pinhole_height = pinhole_model.getHeight();
  float cx_rel = pinhole_model.c[0] / static_cast<float>(pinhole_width);
  float cy_rel = pinhole_model.c[1] / static_cast<float>(pinhole_height);
  rm::Interval pinhole_range = pinhole_model.range;


  // O1Dn model presets
  rm::Interval o1dn_range = o1dn_model.range;

  // OnDn model presets
  rm::Interval ondn_range = ondn_model.range;

  int model_selected = 0;

  // o1dn model 
  rm_spherical_sim.setModel(spherical_model);
  rm_pinhole_sim.setModel(pinhole_model);
  rm_o1dn_sim.setModel(o1dn_model);
  rm_ondn_sim.setModel(ondn_model);
  

  #if defined(WITH_EMBREE)
    using ResultT = rm::Bundle<
        rm::Hits<rm::RAM>,
        rm::Ranges<rm::RAM>,
        rm::Points<rm::RAM>,
        rm::Normals<rm::RAM>
    >;
  #elif defined(WITH_OPTIX)
    using ResultT = rm::Bundle<
        rm::Hits<rm::VRAM_CUDA>,
        rm::Ranges<rm::VRAM_CUDA>,
        rm::Points<rm::VRAM_CUDA>,
        rm::Normals<rm::VRAM_CUDA>
    >;
  #elif defined(WITH_VULKAN)
    using ResultT = rm::Bundle<
        rm::Hits<rm::DEVICE_LOCAL_VULKAN>,
        rm::Ranges<rm::DEVICE_LOCAL_VULKAN>,
        rm::Points<rm::DEVICE_LOCAL_VULKAN>,
        rm::Normals<rm::DEVICE_LOCAL_VULKAN>
    >;
  #else
    #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
  #endif

  polyscope::PointCloud* poly_pcl = nullptr;

  polyscope::state::userCallback = [&](){
    
    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_AutoSelectNewTabs;
    if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) 
    {
      if(ImGui::BeginTabItem("Spherical"))
      {
        model_selected = 0;
        bool spherical_changed = false;
        spherical_changed |= ImGui::DragFloat("FoV", &fov, 0.1f);
        spherical_changed |= ImGui::SliderInt("Scan Lines", &scan_lines, 16, 128);
        spherical_changed |= ImGui::SliderInt("Points per line", &line_points, 1, 1024);
        spherical_changed |= ImGui::DragFloat("Min Range", &spherical_range.min, 0.01f);
        spherical_changed |= ImGui::DragFloat("Max Range", &spherical_range.max, 0.1f);

        if(spherical_changed)
        {
          // Vertical settings
          float inc_deg = fov / (scan_lines - 1);
          float min_deg = -fov / 2.0;
          spherical_model.phi.min = min_deg * DEG_TO_RAD_F;
          spherical_model.phi.inc = inc_deg * DEG_TO_RAD_F;
          spherical_model.phi.size = scan_lines;

          // Horizontal settings
          float h_inc = (2.0 * M_PI) / line_points;
          spherical_model.theta.inc = h_inc;
          spherical_model.theta.size = line_points;

          // min max scanning ranges
          spherical_model.range = spherical_range;

          rm_spherical_sim.setModel(spherical_model);
        }

        ImGui::EndTabItem();
      }
      if(ImGui::BeginTabItem("Pinhole"))
      {
        model_selected = 1;
        bool pinhole_changed = false;
        pinhole_changed |= ImGui::SliderInt("Width", &pinhole_width, 16, 1024);
        pinhole_changed |= ImGui::SliderInt("Height", &pinhole_height, 16, 1024);
        pinhole_changed |= ImGui::DragFloat("Focal X", &fx, 0.1f);
        pinhole_changed |= ImGui::DragFloat("Focal Y", &fy, 0.1f);
        pinhole_changed |= ImGui::SliderFloat("Center X (rel)", &cx_rel, 0.0f, 1.0f);
        pinhole_changed |= ImGui::SliderFloat("Center Y (rel)", &cy_rel, 0.0f, 1.0f);
        pinhole_changed |= ImGui::DragFloat("Min Range", &pinhole_range.min, 0.01f);
        pinhole_changed |= ImGui::DragFloat("Max Range", &pinhole_range.max, 0.1f);

        if(pinhole_changed)
        {
          pinhole_model.width = pinhole_width;
          pinhole_model.height = pinhole_height;
          pinhole_model.f[0] = fx;
          pinhole_model.f[1] = fy;
          pinhole_model.c[0] = cx_rel * static_cast<float>(pinhole_width);
          pinhole_model.c[1] = cy_rel * static_cast<float>(pinhole_height);
          pinhole_model.range = pinhole_range;
          rm_pinhole_sim.setModel(pinhole_model);
        }
        
        ImGui::EndTabItem();
      }
      if(ImGui::BeginTabItem("O1Dn"))
      {
        model_selected = 2;
        bool o1dn_changed = false;

        o1dn_changed |= ImGui::DragFloat("Min Range", &o1dn_range.min, 0.01f);
        o1dn_changed |= ImGui::DragFloat("Max Range", &o1dn_range.max, 0.1f);
        // TODO: load some file?

        if(o1dn_changed)
        {
          o1dn_model.range = o1dn_range;
          rm_o1dn_sim.setModel(o1dn_model);
        }


        if(ImGui::Button("Ouster Meta"))
        {
          auto selection = pfd::open_file("Open Ouster Meta File", ".", {
            "Json Files", "*.json",
          }).result();
          if(!selection.empty())
          {
            std::string ouster_filename = selection[0];
            o1dn_model = rm::o1dn_from_ouster_meta_file(ouster_filename);
            o1dn_model.range = o1dn_range;
            rm_o1dn_sim.setModel(o1dn_model);
          }
        }

        ImGui::EndTabItem();
      }
      if(ImGui::BeginTabItem("OnDn"))
      {
        model_selected = 3;
        bool ondn_changed = false;
        
        ondn_changed |= ImGui::DragFloat("Min Range", &ondn_range.min, 0.01f);
        ondn_changed |= ImGui::DragFloat("Max Range", &ondn_range.max, 0.1f);

        // TODO: load some file?
        if(ondn_changed)
        {
          ondn_model.range = ondn_range;
          rm_ondn_sim.setModel(ondn_model);
        }
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }

    ImGui::Separator();

    if(ImGui::Button("Open Scene File"))
    {
      auto selection = pfd::open_file("Select a mesh or scene file", ".").result();
      if(!selection.empty())
      {
        std::cout << "Loading scene from '" << selection[0] << "'" << std::endl;
        
        #if defined(WITH_EMBREE)
          rm_map = rm::import_embree_map(selection[0]);
        #elif defined(WITH_OPTIX)
          rm_map = rm::import_optix_map(selection[0]);
        #elif defined(WITH_VULKAN)
          rm_map = rm::import_vulkan_map(selection[0]);
        #else
          #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
        #endif
        rm_spherical_sim.setMap(rm_map);
        rm_pinhole_sim.setMap(rm_map);
        rm_o1dn_sim.setMap(rm_map);
        rm_ondn_sim.setMap(rm_map);

        for(auto [rm_id, poly_mesh] : poly_scene)
        {
          polyscope::removeStructure(poly_mesh, true);
        }
        poly_scene.clear();
        #if defined(WITH_EMBREE)
          poly_scene = polyscope_scene_from_rmagine(rm_map->scene);
        #elif defined(WITH_OPTIX)
          poly_scene = polyscope_scene_from_rmagine(rm_map->scene());
        #elif defined(WITH_VULKAN)
          poly_scene = polyscope_scene_from_rmagine(rm_map->scene());
        #else
          #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
        #endif
      }
    }
  };

  while(!polyscope::windowRequestsClose())
  {
    // synchronize acceleration structure
    #if defined(WITH_EMBREE)
      synchronize(poly_scene, rm_map->scene);
    #elif defined(WITH_OPTIX)
      synchronize(poly_scene, rm_map->scene());
    #elif defined(WITH_VULKAN)
      synchronize(poly_scene, rm_map->scene());
    #else
      #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
    #endif

    // update scanner transform
     // Transform from sensor to world, i.e. pose of the sensor
    rm::Transform Tsw = rm::Transform::Identity();

    if(poly_pcl)
    {      
      rm::Matrix4x4 M = rm_from_glm(poly_pcl->getTransform());
      rm::Vector3 s; // scale. not used
      rm::decompose(M, Tsw, s);
      // poly_pcl->rescaleToUnit(); // gives weird results
      // better:
      poly_pcl->setTransform(glm_from_rm(Tsw));
    }


    std::vector<rm::Point> points_filtered;
    std::vector<rm::Vector3> normals_filtered;
    {
      // actual simulation    
      ResultT results;
      if(model_selected == 0) {
        //optix crashes here - idk why
        results = rm_spherical_sim.simulate<ResultT>(Tsw);
      } else if(model_selected == 1) {
        results = rm_pinhole_sim.simulate<ResultT>(Tsw);
      } else if(model_selected == 2) {
        results = rm_o1dn_sim.simulate<ResultT>(Tsw);
      } else if(model_selected == 3) {
        results = rm_ondn_sim.simulate<ResultT>(Tsw);
      }

      #if defined(WITH_EMBREE)
        for(size_t i=0; i<results.points.size(); i++)
        {
          if(results.hits[i] > 0)
          {
            points_filtered.push_back(results.points[i]);
            normals_filtered.push_back(results.normals[i]);
          }
        }
      #elif defined(WITH_OPTIX)
        rm::Memory<rm::Point, rm::RAM> points_ram(results.points.size());
        points_ram = results.points;
        rm::Memory<rm::Vector3, rm::RAM> normals_ram(results.normals.size());
        normals_ram = results.normals;
        rm::Memory<uint8_t, rm::RAM> hits_ram(results.hits.size());
        hits_ram = results.hits;
        for(size_t i=0; i<points_ram.size(); i++)
        {
          if(hits_ram[i] > 0)
          {
            points_filtered.push_back(points_ram[i]);
            normals_filtered.push_back(normals_ram[i]);
          }
        }
      #elif defined(WITH_VULKAN)
        rm::Memory<rm::Point, rm::RAM> points_ram(results.points.size());
        points_ram = results.points;
        rm::Memory<rm::Vector3, rm::RAM> normals_ram(results.normals.size());
        normals_ram = results.normals;
        rm::Memory<uint8_t, rm::RAM> hits_ram(results.hits.size());
        hits_ram = results.hits;
        for(size_t i=0; i<points_ram.size(); i++)
        {
          if(hits_ram[i] > 0)
          {
            points_filtered.push_back(points_ram[i]);
            normals_filtered.push_back(normals_ram[i]);
          }
        }
      #else
        #error Either WITH_EMBREE or WITH_OPTIX or WITH_VULKAN has to be defined // compile time error
      #endif
      // std::cout << "Render " << points_filtered.size() << " points" << std::endl;
    }
    

    if(!poly_pcl)
    {
      // create new pcl
      poly_pcl = polyscope::registerPointCloud("Sensor", points_filtered);
      // choosing quads as default since it allows to render more points 
      poly_pcl->setPointRenderMode(polyscope::PointRenderMode::Quad);
      // add normals
      poly_pcl->addVectorQuantity("Normals", normals_filtered, polyscope::VectorType::STANDARD);
    } 
    else if(points_filtered.size() != poly_pcl->nPoints())
    {
      // recreate PCL
      auto T = poly_pcl->getTransform();
      poly_pcl = polyscope::registerPointCloud("Sensor", points_filtered);
      poly_pcl->setTransform(T);
      // add normals
      poly_pcl->addVectorQuantity("Normals", normals_filtered, polyscope::VectorType::STANDARD);
    }
    else
    {
      // update existing PCL
      poly_pcl->updatePointPositions(points_filtered);
      // add normals
      poly_pcl->addVectorQuantity("Normals", normals_filtered, polyscope::VectorType::STANDARD);
    }

    polyscope::frameTick(); // renders one UI frame, returns immediately
  }

  return 0;
}