#include <array>



#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/PinholeSimulatorEmbree.hpp>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/map/embree/embree_shapes.h>

#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <rmagine/simulation/PinholeSimulatorOptix.hpp>
#include <rmagine/simulation/O1DnSimulatorOptix.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <rmagine/map/OptixMap.hpp>
#include <rmagine/map/optix/OptixInst.hpp>
#include <rmagine/map/optix/optix_shapes.h>

#include <rmagine/simulation/SphereSimulatorVulkan.hpp>
#include <rmagine/simulation/PinholeSimulatorVulkan.hpp>
#include <rmagine/simulation/O1DnSimulatorVulkan.hpp>
#include <rmagine/simulation/OnDnSimulatorVulkan.hpp>
#include <rmagine/map/VulkanMap.hpp>
#include <rmagine/map/vulkan/vulkan_shapes.hpp>



#include <rmagine/types/sensors.h>
#include <rmagine/types/mesh_types.h>
#include <rmagine/math/linalg.h>

#include <rmagine/types/ouster_sensors.h>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"

#include "portable-file-dialogs.h"


namespace rm = rmagine;

rm::SphericalModel generate_default_spherical_model()
{
  return rm::vlp16_900();
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
  rm::EmbreeMapPtr rm_map_embree;
  rm::OptixMapPtr rm_map_optix;
  rm::VulkanMapPtr rm_map_vulkan;
  PolyscopeScene poly_scene;

  // construct simulators
  rm::Transform Tsb = rm::Transform::Identity();

  rm::SphereSimulatorEmbree rm_spherical_sim_embree;
  rm::SphereSimulatorOptix rm_spherical_sim_optix;
  rm::SphereSimulatorVulkan rm_spherical_sim_vulkan;

  rm_spherical_sim_embree.setTsb(Tsb);
  rm_spherical_sim_optix.setTsb(Tsb);
  rm_spherical_sim_vulkan.setTsb(Tsb);

  if(argc < 2)
  {
    throw std::runtime_error("First arg neds to be a path to a map file");
  }
  else
  {
    rm_map_embree = rm::import_embree_map(argv[1]);
    rm_map_optix = rm::import_optix_map(argv[1]);
    rm_map_vulkan = rm::import_vulkan_map(argv[1]);
  }

  rm_spherical_sim_embree.setMap(rm_map_embree);
  rm_spherical_sim_optix.setMap(rm_map_optix);
  rm_spherical_sim_vulkan.setMap(rm_map_vulkan);

  poly_scene = polyscope_scene_from_rmagine(rm_map_embree->scene);

  rm::SphericalModel spherical_model = generate_default_spherical_model();

  rm_spherical_sim_embree.setModel(spherical_model);
  rm_spherical_sim_optix.setModel(spherical_model);
  rm_spherical_sim_vulkan.setModel(spherical_model);

  using ResultTEmbree = rm::Bundle<
    rm::Hits<rm::RAM>,
    rm::Points<rm::RAM>,
    rm::Normals<rm::RAM>
  >;

  using ResultTOptix = rm::Bundle<
    rm::Hits<rm::VRAM_CUDA>,
    rm::Points<rm::VRAM_CUDA>,
    rm::Normals<rm::VRAM_CUDA>
  >;

  using ResultTVulkan = rm::Bundle<
    rm::Hits<rm::DEVICE_LOCAL_VULKAN>,
    rm::Points<rm::DEVICE_LOCAL_VULKAN>,
    rm::Normals<rm::DEVICE_LOCAL_VULKAN>
  >;

  polyscope::PointCloud* poly_pcl_embree = nullptr;
  polyscope::PointCloud* poly_pcl_optix = nullptr;
  polyscope::PointCloud* poly_pcl_vulkan = nullptr;

  // update scanner transform
  // Transform from sensor to world, i.e. pose of the sensor
  rm::Transform Tsw = rm::Transform::Identity();



  while(!polyscope::windowRequestsClose())
  {
    //no sync, maps dont change anyways

    // update scanner transform
    // Transform from sensor to world, i.e. pose of the sensor
    rm::Transform Tsw_embree = rm::Transform::Identity();
    rm::Transform Tsw_optix = rm::Transform::Identity();
    rm::Transform Tsw_vulkan = rm::Transform::Identity();

    if(poly_pcl_embree)
    {      
      rm::Matrix4x4 M = rm_from_glm(poly_pcl_embree->getTransform());
      rm::Vector3 s; // scale. not used
      rm::decompose(M, Tsw_embree, s);
      // poly_pcl->rescaleToUnit(); // gives weird results
      // better:
      poly_pcl_embree->setTransform(glm_from_rm(Tsw_embree));
    }

    if(poly_pcl_optix)
    {      
      rm::Matrix4x4 M = rm_from_glm(poly_pcl_optix->getTransform());
      rm::Vector3 s; // scale. not used
      rm::decompose(M, Tsw_optix, s);
      // poly_pcl->rescaleToUnit(); // gives weird results
      // better:
      poly_pcl_optix->setTransform(glm_from_rm(Tsw_optix));
    }

    if(poly_pcl_vulkan)
    {      
      rm::Matrix4x4 M = rm_from_glm(poly_pcl_vulkan->getTransform());
      rm::Vector3 s; // scale. not used
      rm::decompose(M, Tsw_vulkan, s);
      // poly_pcl->rescaleToUnit(); // gives weird results
      // better:
      poly_pcl_vulkan->setTransform(glm_from_rm(Tsw_vulkan));
    }

    std::vector<rm::Point> points_filtered_embree;
    std::vector<rm::Vector3> normals_filtered_embree;

    std::vector<rm::Point> points_filtered_optix;
    std::vector<rm::Vector3> normals_filtered_optix;

    std::vector<rm::Point> points_filtered_vulkan;
    std::vector<rm::Vector3> normals_filtered_vulkan;

    // actual simulation
    {
      ResultTEmbree results_embree;
      ResultTOptix results_optix;
      ResultTVulkan results_vulkan;

      results_embree = rm_spherical_sim_embree.simulate<ResultTEmbree>(Tsw);

      rm::resize_memory_bundle<rm::VRAM_CUDA, ResultTOptix>(results_optix, spherical_model.getWidth(), spherical_model.getHeight(), 1);
      rm_spherical_sim_optix.simulate<ResultTOptix>(Tsw, results_optix);

      rm::resize_memory_bundle<rm::DEVICE_LOCAL_VULKAN,ResultTVulkan>(results_vulkan, spherical_model.getWidth(), spherical_model.getHeight(), 1);
      rm_spherical_sim_vulkan.simulate<ResultTVulkan>(Tsw, results_vulkan);



      for(size_t i=0; i<results_embree.points.size(); i++)
      {
        if(results_embree.hits[i] > 0)
        {
          points_filtered_embree.push_back(results_embree.points[i]);
          normals_filtered_embree.push_back(results_embree.normals[i]);
        }
      }

      rm::Memory<rm::Point, rm::RAM> points_optix_ram(results_optix.points.size());
      points_optix_ram = results_optix.points;
      rm::Memory<rm::Vector3, rm::RAM> normals_optix_ram(results_optix.normals.size());
      normals_optix_ram = results_optix.normals;
      rm::Memory<uint8_t, rm::RAM> hits_optix_ram(results_optix.hits.size());
      hits_optix_ram = results_optix.hits;
      for(size_t i=0; i<points_optix_ram.size(); i++)
      {
        if(hits_optix_ram[i] > 0)
        {
          points_filtered_optix.push_back(points_optix_ram[i]);
          normals_filtered_optix.push_back(normals_optix_ram[i]);
        }
      }

      rm::Memory<rm::Point, rm::RAM> points_vulkan_ram(results_vulkan.points.size());
      points_vulkan_ram = results_vulkan.points;
      rm::Memory<rm::Vector3, rm::RAM> normals_vulkan_ram(results_vulkan.normals.size());
      normals_vulkan_ram = results_vulkan.normals;
      rm::Memory<uint8_t, rm::RAM> hits_vulkan_ram(results_vulkan.hits.size());
      hits_vulkan_ram = results_vulkan.hits;
      for(size_t i=0; i<points_vulkan_ram.size(); i++)
      {
        if(hits_vulkan_ram[i] > 0)
        {
          points_filtered_vulkan.push_back(points_vulkan_ram[i]);
          normals_filtered_vulkan.push_back(normals_vulkan_ram[i]);
        }
      }



      if(!poly_pcl_embree)
      {
        // create new pcl
        poly_pcl_embree = polyscope::registerPointCloud("SensorEmbree", points_filtered_embree);
        // choosing quads as default since it allows to render more points 
        poly_pcl_embree->setPointRenderMode(polyscope::PointRenderMode::Quad);
        // add normals
        poly_pcl_embree->addVectorQuantity("NormalsEmbree", normals_filtered_embree, polyscope::VectorType::STANDARD);
      }
      else
      {
        // update existing PCL
        poly_pcl_embree->updatePointPositions(points_filtered_embree);
        // add normals
        poly_pcl_embree->addVectorQuantity("NormalsEmbree", normals_filtered_embree, polyscope::VectorType::STANDARD);
      }

      if(!poly_pcl_optix)
      {
        // create new pcl
        poly_pcl_optix = polyscope::registerPointCloud("SensorOptix", points_filtered_optix);
        // choosing quads as default since it allows to render more points 
        poly_pcl_optix->setPointRenderMode(polyscope::PointRenderMode::Quad);
        // add normals
        poly_pcl_optix->addVectorQuantity("NormalsOptix", normals_filtered_optix, polyscope::VectorType::STANDARD);
      }
      else
      {
        // update existing PCL
        poly_pcl_optix->updatePointPositions(points_filtered_optix);
        // add normals
        poly_pcl_optix->addVectorQuantity("NormalsOptix", normals_filtered_optix, polyscope::VectorType::STANDARD);
      }

      if(!poly_pcl_vulkan)
      {
        // create new pcl
        poly_pcl_vulkan = polyscope::registerPointCloud("SensorVulkan", points_filtered_vulkan);
        // choosing quads as default since it allows to render more points 
        poly_pcl_vulkan->setPointRenderMode(polyscope::PointRenderMode::Quad);
        // add normals
        poly_pcl_vulkan->addVectorQuantity("NormalsVulkan", normals_filtered_vulkan, polyscope::VectorType::STANDARD);
      }
      else
      {
        // update existing PCL
        poly_pcl_vulkan->updatePointPositions(points_filtered_vulkan);
        // add normals
        poly_pcl_vulkan->addVectorQuantity("NormalsVulkan", normals_filtered_vulkan, polyscope::VectorType::STANDARD);
      }
    }

    polyscope::frameTick(); // renders one UI frame, returns immediately
  }

  return 0;
}