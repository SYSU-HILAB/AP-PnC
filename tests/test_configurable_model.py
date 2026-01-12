#!/usr/bin/env python3

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from feature.lu_mpc.model.sim.sim_simple_model import export_drone_model, DroneModelConfig, DEFAULT_CONFIG

def test_structured_config():
    """Test the clean structured configuration approach"""

    print("🚀 Testing Clean Structured Configuration")
    print("=" * 45)

    # 1. Test with default config
    print("\n1️⃣ Testing with default configuration...")
    model = export_drone_model()
    print('✓ Default configuration works')
    print(f'   Default config: {DEFAULT_CONFIG}')

    # 2. Test with custom config object
    print("\n2️⃣ Testing with custom configuration object...")
    custom_config = DroneModelConfig(
        tau_at=0.2,
        tau_omega=0.1,
        mass=2.0,
        g=-9.81,
        drag_coeff=0.2
    )
    model = export_drone_model(custom_config)
    print('✓ Custom configuration object works')
    print(f'   Custom config: {custom_config}')

    # 3. Test partial config creation (only specify needed params)
    print("\n3️⃣ Testing partial configuration...")
    lightweight_config = DroneModelConfig(mass=0.8, drag_coeff=0.05)
    model = export_drone_model(lightweight_config)
    print('✓ Partial configuration works')
    print(f'   Lightweight config: {lightweight_config}')

    # 4. Test different use cases
    print("\n4️⃣ Testing different use cases...")

    # Heavy drone
    heavy_config = DroneModelConfig(mass=5.0, tau_at=0.3, tau_omega=0.15)
    model = export_drone_model(heavy_config)
    print('✓ Heavy drone configuration works')

    # High drag drone
    high_drag_config = DroneModelConfig(drag_coeff=0.5, mass=1.2)
    model = export_drone_model(high_drag_config)
    print('✓ High drag configuration works')

    # Mars gravity drone
    mars_config = DroneModelConfig(g=-3.71, mass=0.5, drag_coeff=0.02)
    model = export_drone_model(mars_config)
    print('✓ Mars gravity configuration works')

    print('\n🎉 All tests passed! Structured configuration is clean and consistent!')
    print('\n📋 Usage Summary:')
    print('   • export_drone_model()               # Use defaults')
    print('   • export_drone_model(config_obj)     # Use config object')
    print('   • DroneModelConfig(param1=value)     # Create partial config')
    print('\n✨ Benefits: Type-safe, self-documenting, consistent!')

if __name__ == "__main__":
    test_structured_config()