# SceneGraphs.jl
Scene Graphs for abstract (acyclic) representation of 3D Worlds. Designed for 3D engines built upon [OpenGL](https://www.opengl.org/), [DirectX](https://developer.nvidia.com/directx), or [Vulkan](https://www.vulkan.org/).

*SceneGraphs* is a relatively complex project under the hood, exposing a simplified API to operate with in your applications. Refer to the [documentation](https://kiruse.dev/docs/SceneGraphs.jl) for more details.

# Example
```julia
using SceneGraphs

abstract type AbstractEntity end

struct Entity <: AbstractEntity
    transform::SpriteTransform{Entity, Float64}
end
Entity() = Entity(SpriteTransform())

let scene = Scene3D(), ntt1 = Entity(), ntt2 = Entity(), ntt3 = Entity()
    push!(scene, ntt1)
    
    parent!(ntt3, ntt2)
    parent!(ntt2, ntt1)
    
    translate!(ntt1, 1, 2, 3)
    translate!(ntt2, 2, 3, 4)
    rotate!(ntt2, deg2rad(45))
    scale!(ntt3, 2)
    
    update!(scene)
end
```
