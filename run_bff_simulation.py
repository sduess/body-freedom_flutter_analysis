import numpy as np
from bff_flying_wing import BFF_Flying_Wing
from helper_functions.get_settings import get_settings
#from helper_functions.get_cs_type import get_cs_type


cases_route = '../01_case_files/01_case_files/'
output_route = './output/'

case_name = 'bff_test_cs'


# Trim values for SuperFLEXOP
trim_values = {'alpha': 4.792613660659094527e-02,
                'delta': -2.664350980482567688e-01,
                'thrust': -1.325090000148728686e-01,
                }




linearize = False
use_trim = False
use_rom = False 
convection_scheme = 2

flow = [
        'BeamLoader', 
        'AerogridLoader',
        'Modal',
        'AerogridPlot',
        'BeamPlot',
        'StaticUvlm',
        'AeroForcesCalculator',
        'StaticCoupled',
        'AeroForcesCalculator',
        'StaticTrim',     
        'AerogridPlot',
        'BeamPlot',
        'DynamicCoupled',
        'Modal',
        'LinearAssembler',
        'AsymptoticStability',
        'SaveData',
]
print(flow)
if not use_trim:
    flow.remove('StaticTrim')
else:
    flow.remove('StaticCoupled')

if 'LinearAssembler' in flow:
    unsteady_force_distribution = False
else:
    unsteady_force_distribution = True
     
# Set cruise parameter
alpha = trim_values['alpha'] # rad
u_inf = 15 # 5- 20 m/s
rho = 1.225
gravity =  True
horseshoe =  False
free_flight = True
wake_length = 10
cs_deflection = trim_values['delta'] # rad
trim_cs_index = 0
cs_deflection_initial = cs_deflection 
thrust = trim_values['thrust'] 
num_modes = 41
num_chord_panels = 8
n_elem_multiplier = 4

# Init FLEXOP Model
bff_model = BFF_Flying_Wing(case_name, cases_route, output_route)
bff_model.init_aeroelastic(m=num_chord_panels,
                           n_elem_multiplier=n_elem_multiplier,
                           init_cs_deflection=cs_deflection_initial)
bff_model.structure.set_thrust(thrust)
#  Simulation settings
CFL = 1
dt = CFL * bff_model.structure.chord_wing / bff_model.aero.m / u_inf
# numerics
n_step = 5
structural_relaxation_factor = 0.6
relaxation_factor = 0.2
tolerance = 1e-6
fsi_tolerance = 1e-4
num_cores = 4
newmark_damp = 0.5e-4
n_tstep = 1

# Gust velocity field
gust = True
gust_settings=None
if gust:
        gust_settings  ={'gust_shape': '1-cos',
                        'gust_length': 10.,
                        'gust_intensity': 0.01,
                        'gust_offset': 10 * dt *u_inf}  
# ROM settings
rom_settings = {
    'use': use_rom,
    'rom_method': 'Krylov',
    'rom_method_settings': {'Krylov': {
                                        'algorithm': 'mimo_rational_arnoldi',
                                        'r': 4, 
                                        'frequency': np.array([0]),
                                        'single_side': 'observability',
                                        },
                           }
                }
scaling_dict = {'length':bff_model.structure.chord_wing / 2., 'speed': u_inf, 'density': rho}
# Get settings dict
settings = get_settings(bff_model,
                        flow,
                        dt,
                        alpha = alpha,
                        cs_deflection_initial = cs_deflection,
                        u_inf = u_inf,
                        rho = rho,
                        free_flight=free_flight,
                        thrust = thrust,
                        wake_length = wake_length,
                        num_cores = num_cores,
                        tolerance = tolerance,
                        fsi_tolerance = fsi_tolerance,
                        structural_relaxation_factor = structural_relaxation_factor,
                        newmark_damp = newmark_damp,
                        n_tstep = n_tstep,
                        num_modes = num_modes,
                        rom_settings = rom_settings,                      
                        gust = gust,
                        gust_settings = gust_settings,
                        convection_scheme=convection_scheme,
                        trim_cs_index=trim_cs_index,
                        unsteady_force_distribution=unsteady_force_distribution,
                        scaling_dict=scaling_dict)
# print(settings)
# Generate finale FLEXOP model
bff_model.generate()
bff_model.create_settings(settings)
bff_model.structure.calculate_aircraft_mass()
bff_model.run()


