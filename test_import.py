import importlib.util
import os

# Ensure the src directory is in PYTHONPATH
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

# Check if the module can be found
spec = importlib.util.find_spec('q_learning')
print(f"Module spec: {spec}")

if spec is not None:
    q_learning = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(q_learning)
    
    # Verify if QLearningAgent is available
    try:
        from q_learning import QLearningAgent
        print("QLearningAgent imported successfully")
    except ImportError as e:
        print(f"ImportError: {e}")
else:
    print("q_learning module not found")