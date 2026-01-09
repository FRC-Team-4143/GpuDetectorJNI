#include <iostream>
#include <cassert>

// Simplified test to verify handle management
struct PoseFusion {};

static PoseFusion* pose_fusions[10] = {nullptr};

long createPoseFusion() {
    // Find first available slot
    for (int i = 0; i < 10; ++i) {
        if (pose_fusions[i] == nullptr) {
            pose_fusions[i] = new PoseFusion();
            std::cout << "Created pose fusion object: " << i << std::endl;
            return i;
        }
    }
    
    std::cout << "createPoseFusion: too many pose fusion objects" << std::endl;
    return -1;
}

void destroyPoseFusion(long handle) {
    if (handle < 0 || handle >= 10 || !pose_fusions[handle]) {
        std::cout << "destroyPoseFusion: invalid handle" << std::endl;
        return;
    }
    
    delete pose_fusions[handle];
    pose_fusions[handle] = nullptr;
    std::cout << "Destroyed pose fusion object: " << handle << std::endl;
}

int main() {
    std::cout << "Testing handle management..." << std::endl;
    
    // Create 10 objects (should fill all slots)
    long handles[10];
    for (int i = 0; i < 10; ++i) {
        handles[i] = createPoseFusion();
        assert(handles[i] == i);
    }
    
    // Try to create 11th object (should fail)
    long handle11 = createPoseFusion();
    assert(handle11 == -1);
    std::cout << "Correctly rejected 11th object" << std::endl;
    
    // Destroy objects 3 and 7
    destroyPoseFusion(handles[3]);
    destroyPoseFusion(handles[7]);
    
    // Create new objects (should reuse slots 3 and 7)
    long new_handle1 = createPoseFusion();
    long new_handle2 = createPoseFusion();
    
    assert(new_handle1 == 3 || new_handle1 == 7);
    assert(new_handle2 == 3 || new_handle2 == 7);
    assert(new_handle1 != new_handle2);
    
    std::cout << "Successfully reused handles " << new_handle1 << " and " << new_handle2 << std::endl;
    
    // Cleanup
    for (int i = 0; i < 10; ++i) {
        if (pose_fusions[i]) {
            destroyPoseFusion(i);
        }
    }
    
    std::cout << "All handle management tests PASSED!" << std::endl;
    return 0;
}
