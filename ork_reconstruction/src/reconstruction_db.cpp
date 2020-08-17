#include <string>

#include <boost/format.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/opencv.h>

using ecto::tendrils;

namespace reconstruction
{
  void
  insert_mesh(const std::string& db_url, const std::string& object_id, const std::string& session_id,
              const std::string& mesh_file, const std::string& surfel_file)
  {
    using namespace object_recognition_core::db;
    ObjectDbParameters params(ObjectDbParameters::COUCHDB);
    params.set_parameter("root", db_url);
    ObjectDbPtr db = params.generateDb();

    Document doc;
    doc.set_db(db);
    doc.set_document_id("meshes");
    doc.load_fields();

    std::ifstream mesh_stream(mesh_file.c_str());
    doc.set_attachment_stream("mesh.ply", mesh_stream);
    std::ifstream surfel_stream(surfel_file.c_str());
    doc.set_attachment_stream("surfel.ply", surfel_stream);
    doc.set_field("object_id", object_id);
    doc.set_field("session_id", session_id);
    doc.Persist();
  }
}
