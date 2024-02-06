import capytaine as cpt
import logging
logging.basicConfig(level=logging.INFO)

mesh = cpt.load_mesh("hydrus/hydrus_description/meshes/base_link.STL", file_format="stl")
body = cpt.FloatingBody(mesh,
                        dofs=cpt.rigid_body_dofs(rotation_center=(0, 0, 0)),
                        center_of_mass=(0, 0, 0))

hydrostatics = body.compute_hydrostatics(rho=1025.0)

print(hydrostatics["disp_volume"])
# 3.82267415555807

print(hydrostatics["hydrostatic_stiffness"])
# <xarray.DataArray 'hydrostatic_stiffness' (influenced_dof: 7, radiating_dof: 7)>
# [...]
# Coordinates:
#   * influenced_dof  (influenced_dof) <U7 'Surge' 'Sway' ... 'Yaw' 'x-shear'
#   * radiating_dof   (radiating_dof) <U7 'Surge' 'Sway' ... 'Yaw' 'x-shear'

print(hydrostatics["inertia_matrix"])
# <xarray.DataArray 'inertia_matrix' (influenced_dof: 7, radiating_dof: 7)>
# [...]
# Coordinates:
#   * influenced_dof  (influenced_dof) <U7 'Surge' 'Sway' ... 'Yaw' 'x-shear'
#   * radiating_dof   (radiating_dof) <U7 'Surge' 'Sway' ... 'Yaw' 'x-shear'