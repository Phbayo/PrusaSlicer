///|/ Copyright (c) Prusa Research 2022 - 2023 Oleksandra Iushchenko @YuSanka, Enrico Turri
///@enricoturri1966, Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "libslic3r/libslic3r.h"
#include "GLGizmoSlaBase.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosCommon.hpp"
#include "slic3r/GUI/MsgDialog.hpp"
#include "slic3r/GUI/MainFrame.hpp"
#include "slic3r/GUI/BackgroundSlicingProcess.hpp"

#include <thread>
#include "libslic3r/MultipleBeds.hpp"
namespace Slic3r { namespace GUI {

static const ColorRGBA DISABLED_COLOR = ColorRGBA::DARK_GRAY();
static const int VOLUME_RAYCASTERS_BASE_ID = (int) SceneRaycaster::EIdBase::Gizmo;

GLGizmoSlaBase::GLGizmoSlaBase(
    GLCanvas3D &parent,
    const std::string &icon_filename,
    unsigned int sprite_id,
    SLAPrintObjectStep min_step
)
    : GLGizmoBase(parent, icon_filename, sprite_id), m_min_sla_print_object_step((int) min_step) {}

/*static*/ bool GLGizmoSlaBase::selected_print_object_exists(
    const GLCanvas3D &canvas, const wxString &text
) {
    /*if (const Selection& sel = canvas.get_selection(); !sel.is_single_full_instance() ||
    !sel.get_model()->objects[sel.get_object_idx()]
        || !
    canvas.sla_print()->get_print_object_by_model_object_id(sel.get_model()->objects[sel.get_object_idx()]->id()))
    {
        if (! text.IsEmpty())
            wxGetApp().CallAfter([text]() {
                MessageDialog dlg(GUI::wxGetApp().mainframe, text,
                    _L("Bed selection mismatch"), wxICON_INFORMATION | wxOK);
                dlg.ShowModal();
            });
        return false;
    }*/
    return true;
}

void GLGizmoSlaBase::reslice_until_step(SLAPrintObjectStep step, bool postpone_error_messages) {
    // 最朴素的 直接while到step_done
    wxGetApp().CallAfter([this, step, postpone_error_messages]() {
        const Selection &selection = m_parent.get_selection();
        std::set<unsigned int> mo_idxs = selection.get_object_idxs();
        for (const auto &id : mo_idxs) {
            ModelObject *mo = selection.get_model()->objects[id];
            wxGetApp().plater()->reslice_SLA_until_step(step, *mo, true);
            const SLAPrintObject *po = m_parent.sla_print()->get_print_object_by_model_object_id(
                mo->id()
            );
            while (!po->is_step_done(step)) {}
            // std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // wxGetApp().CallAfter([this, step, postpone_error_messages]() {
    //     const Selection &selection = m_parent.get_selection();
    //     std::vector<ModelObject *> objects;
    //     objects.reserve(selection.get_object_idxs().size());
    //     for (unsigned int id : selection.get_object_idxs()) {
    //         if (ModelObject *object = selection.get_model()->objects[id])
    //             objects.push_back(object);
    //     }
    //     if (objects.empty())
    //         return;
    //     Plater *plater = wxGetApp().plater();
    //     if (plater == nullptr)
    //         return;
    //     const SLAPrint *sla_print = m_parent.sla_print();
    //     if (sla_print == nullptr)
    //         return;
    //     auto pending = std::make_shared<std::vector<ModelObject *>>(std::move(objects));
    //     auto advance = std::make_shared<std::function<void(size_t)>>();
    //     auto wait_for_completion = std::make_shared<std::function<void(size_t)>>();
    //     *advance = [pending, plater, sla_print, step, postpone_error_messages, advance,
    //                 wait_for_completion](size_t index) {
    //         if (index >= pending->size())
    //             return;

    //        plater->reslice_SLA_until_step(step, *(*pending)[index], postpone_error_messages);
    //        (*wait_for_completion)(index);
    //    };

    //    *wait_for_completion =
    //        [pending, plater, sla_print, step, advance, wait_for_completion](size_t index) {
    //            if (index >= pending->size())
    //                return;
    //            ModelObject *mo = (*pending)[index];
    //            const SLAPrintObject *po = sla_print->get_print_object_by_model_object_id(mo->id());
    //            if (po != nullptr && !po->is_step_done(step)) {
    //                //if (!plater->p_is_idle() || (po != nullptr && !po->is_step_done(step))) {
    //                    wxGetApp().CallAfter([wait_for_completion, index]() {
    //                    (*wait_for_completion)(index);
    //                });
    //                return;
    //            }
    //            wxGetApp().CallAfter([advance, index]() { (*advance)(index + 1); });
    //        };
    //    (*advance)(0);
    //});
}

CommonGizmosDataID GLGizmoSlaBase::on_get_requirements() const {
    // return CommonGizmosDataID(
    //             int(CommonGizmosDataID::SelectionInfo)
    //           | int(CommonGizmosDataID::InstancesHider)
    //           | int(CommonGizmosDataID::Raycaster)
    //           | int(CommonGizmosDataID::ObjectClipper)
    //           | int(CommonGizmosDataID::SupportsClipper));
    // return CommonGizmosDataID(
    //     int(CommonGizmosDataID::SelectionInfo) |
    //     int(CommonGizmosDataID::Raycaster) | int(CommonGizmosDataID::ObjectClipper) |
    //     int(CommonGizmosDataID::SupportsClipper)
    //);
    return CommonGizmosDataID(0);
}

void GLGizmoSlaBase::update_volumes() {
    m_volumes.clear();
    unregister_volume_raycasters_for_picking();

    const Selection &selection = m_parent.get_selection();

    const Selection::ObjectIdxsToInstanceIdxsMap content = selection.get_content();

    // TODO:一个符合一个不符合时 可能会有问题
    m_input_enabled = false;

    std::set<unsigned int> mo_idxs = selection.get_object_idxs();
    for (auto const object_idx : mo_idxs) {
        ModelObject *mo = selection.get_model()->objects[object_idx];
        if (mo == nullptr)
            return;
        const SLAPrintObject *po = m_parent.sla_print()->get_print_object_by_model_object_id(mo->id(
        ));
        if (po == nullptr)
            return;
        TriangleMesh backend_mesh;
        std::shared_ptr<const indexed_triangle_set> preview_mesh_ptr = po->get_mesh_to_print();
        if (preview_mesh_ptr != nullptr)
            backend_mesh = TriangleMesh(*preview_mesh_ptr);

        if (!backend_mesh.empty()) {
            auto last_comp_step = static_cast<int>(po->last_completed_step());
            if (last_comp_step == slaposCount)
                last_comp_step = -1;

            m_input_enabled = last_comp_step >= m_min_sla_print_object_step ||
                po->model_object()->sla_points_status == sla::PointsStatus::UserModified;

            int instance_idx = 0; // 默认只有一个实例
            const Geometry::Transformation &inst_trafo =
                po->model_object()->instances[instance_idx]->get_transformation();
            const double current_elevation = po->get_current_elevation();
            auto add_volume =
                [this, object_idx, instance_idx, &inst_trafo, current_elevation](
                    const TriangleMesh &mesh, int volume_id, bool add_mesh_raycaster = false
                ) {
                    GLVolume *volume = m_volumes.volumes.emplace_back(new GLVolume());
                    volume->model.init_from(mesh);
                    volume->set_instance_transformation(inst_trafo);
                    volume->set_sla_shift_z(current_elevation);
                    if (add_mesh_raycaster)
                        volume->mesh_raycaster = std::make_unique<GUI::MeshRaycaster>(mesh);
                    if (m_input_enabled)
                        volume->selected = true; // to set the proper color
                    else
                        volume->set_color(DISABLED_COLOR);
                    volume->composite_id = GLVolume::CompositeID(object_idx, volume_id, instance_idx);
                };

            const Transform3d po_trafo_inverse = po->trafo().inverse();
            // main mesh
            backend_mesh.translate(
                s_multiple_beds.get_bed_translation(s_multiple_beds.get_active_bed()).cast<float>()
            );
            backend_mesh.transform(po_trafo_inverse);
            add_volume(backend_mesh, 0, true);
            // supports mesh
            TriangleMesh supports_mesh = po->support_mesh();
            if (!supports_mesh.empty()) {
                supports_mesh.translate(
                    s_multiple_beds.get_bed_translation(s_multiple_beds.get_active_bed())
                        .cast<float>()
                );
                supports_mesh.transform(po_trafo_inverse);
                add_volume(supports_mesh, -int(slaposSupportTree));
            }
            // pad mesh
            TriangleMesh pad_mesh = po->pad_mesh();
            if (!pad_mesh.empty()) {
                pad_mesh.translate(
                    s_multiple_beds.get_bed_translation(s_multiple_beds.get_active_bed())
                        .cast<float>()
                );
                pad_mesh.transform(po_trafo_inverse);
                add_volume(pad_mesh, -int(slaposPad));
            }
        }

        if (m_volumes.volumes.empty()) {
            // No valid mesh found in the backend. Use the selection to duplicate the volumes
            const Selection &selection = m_parent.get_selection();
            const Selection::IndicesList &idxs = selection.get_volume_idxs();
            for (unsigned int idx : idxs) {
                const GLVolume *v = selection.get_volume(idx);
                if (!v->is_modifier) {
                    m_volumes.volumes.emplace_back(new GLVolume());
                    GLVolume *new_volume = m_volumes.volumes.back();
                    const TriangleMesh &mesh = mo->volumes[v->volume_idx()]->mesh();
                    new_volume->model.init_from(mesh);
                    new_volume->set_instance_transformation(v->get_instance_transformation());
                    new_volume->set_volume_transformation(v->get_volume_transformation());
                    new_volume->set_sla_shift_z(v->get_sla_shift_z());
                    new_volume->set_color(DISABLED_COLOR);
                    new_volume->mesh_raycaster = std::make_unique<GUI::MeshRaycaster>(mesh);
                }
            }
        }
    }
    register_volume_raycasters_for_picking();
}

void GLGizmoSlaBase::render_volumes() {
    GLShaderProgram *shader = wxGetApp().get_shader("gouraud_light_clip");
    if (shader == nullptr)
        return;

    shader->start_using();
    shader->set_uniform("emission_factor", 0.0f);
    const Camera &camera = wxGetApp().plater()->get_camera();

    // ClippingPlane clipping_plane = (m_c->object_clipper()->get_position() == 0.0) ?
    // ClippingPlane::ClipsNothing() : *m_c->object_clipper()->get_clipping_plane(); if
    // (m_c->object_clipper()->get_position() != 0.0)
    //     clipping_plane.set_normal(-clipping_plane.get_normal());
    // else
    //     // on Linux the clipping plane does not work when using DBL_MAX
    //     clipping_plane.set_offset(FLT_MAX);
    // m_volumes.set_clipping_plane(clipping_plane.get_data());

    for (GLVolume *v : m_volumes.volumes) {
        v->is_active = m_show_sla_supports || (!v->is_sla_pad() && !v->is_sla_support());
    }

    m_volumes.render(
        GLVolumeCollection::ERenderType::Opaque, true, camera.get_view_matrix(),
        camera.get_projection_matrix()
    );
    shader->stop_using();
}

void GLGizmoSlaBase::register_volume_raycasters_for_picking() {
    // for (size_t i = 0; i < m_volumes.volumes.size(); ++i) {
    //     const GLVolume* v = m_volumes.volumes[i];
    //     if (!v->is_sla_pad() && !v->is_sla_support())
    //         m_volume_raycasters[v->composite_id.object_id] = m_parent.add_raycaster_for_picking(
    //             SceneRaycaster::EType::Volume, v->composite_id.object_id,
    //             *v->mesh_raycaster, v->world_matrix());
    // }
    auto all_volmue = m_parent.get_volumes().volumes;
    std::set<unsigned int> mo_idxs = m_parent.get_selection().get_object_idxs();

    for (const auto v : all_volmue) {
        // const GLVolume *v = m_volumes.volumes[i];
        if (!v->is_sla_pad() && !v->is_sla_support()) {
            if (mo_idxs.find(v->composite_id.object_id) != mo_idxs.end())
                m_volume_raycasters[v->composite_id.object_id] = m_parent.add_raycaster_for_picking(
                    SceneRaycaster::EType::Volume, v->composite_id.object_id, *v->mesh_raycaster,
                    v->world_matrix()
                );
            else {
                // 用gizmo的投射器盖住其他没选中的，这样就不会被拾取到
                m_volume_raycasters[v->composite_id.object_id] = m_parent.add_raycaster_for_picking(
                    SceneRaycaster::EType::Gizmo,
                    VOLUME_RAYCASTERS_BASE_ID + v->composite_id.object_id, *v->mesh_raycaster,
                    v->world_matrix()
                );
            }
        }
    }
}

void GLGizmoSlaBase::unregister_volume_raycasters_for_picking() {
    std::set<unsigned int> mo_idxs = m_parent.get_selection().get_object_idxs();

    // for (const auto &raycaster : m_volume_raycasters) {
    //     if (raycaster.second)
    //         m_parent.remove_raycasters_for_picking(
    //             SceneRaycaster::EType::Volume,
    //             SceneRaycaster::decode_id(SceneRaycaster::EType::Volume, raycaster.second->get_id())
    //         );
    // }

    for (auto &[key, v] : m_volume_raycasters) {
        if (mo_idxs.find(key) == mo_idxs.end())
            m_parent.remove_raycasters_for_picking(
                SceneRaycaster::EType::Gizmo, VOLUME_RAYCASTERS_BASE_ID + key
            );
        else
            m_parent.remove_raycasters_for_picking(SceneRaycaster::EType::Volume, key);
    }

    m_volume_raycasters.clear();
}

// Unprojects the mouse position on the mesh and saves hit point and normal of the facet into
// pos_and_normal Return false if no intersection was found, true otherwise.
bool GLGizmoSlaBase::unproject_on_mesh(
    const Vec2d &mouse_pos, std::pair<Vec3f, Vec3f> &pos_and_normal
) {
    if (m_c->raycaster()->raycasters().size() != 1)
        return false;
    if (!m_c->raycaster()->raycaster())
        return false;
    if (m_volumes.volumes.empty())
        return false;

    auto *inst = m_c->selection_info()->model_instance();
    if (!inst)
        return false;

    Transform3d trafo = m_volumes.volumes.front()->world_matrix();
    if (m_c->selection_info() && m_c->selection_info()->print_object()) {
        double shift_z = m_c->selection_info()->print_object()->get_current_elevation();
        trafo = inst->get_transformation().get_matrix();
        trafo.translation()(2) += shift_z;
    }

    // The raycaster query
    Vec3f hit;
    Vec3f normal;
    if (m_c->raycaster()->raycaster()->unproject_on_mesh(
            mouse_pos, trafo /*m_volumes.volumes.front()->world_matrix()*/,
            wxGetApp().plater()->get_camera(), hit, normal,
            m_c->object_clipper()->get_position() != 0.0 ?
                m_c->object_clipper()->get_clipping_plane() :
                nullptr
        )) {
        // Return both the point and the facet normal.
        pos_and_normal = std::make_pair(hit, normal);
        return true;
    }
    return false;
}

}} // namespace Slic3r::GUI
