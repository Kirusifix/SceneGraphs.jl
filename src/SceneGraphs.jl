######################################################################
# Scene Graphs resemble an abstract representation of 2D & 3D scenes.
# SceneGraphs library is designed to plug into existing systems as a
# component.
# -----
# Licensed under Apache License 2.0
module SceneGraphs
using ExtraFun
using GenerateProperties
using StaticArrays
import LinearAlgebra: I
import Base.Threads: @threads

# ===== Typedefs =====

const Optional{T} = Union{T, Nothing}

export Vector2, Vector3, Vector4
const Vector2{T} = SVector{2, T}
const Vector3{T} = SVector{3, T}
const Vector4{T} = SVector{4, T}

export Matrix2, Matrix3, Matrix4
const Matrix2{T} = SMatrix{2, 2, T}
const Matrix3{T} = SMatrix{3, 3, T}
const Matrix4{T} = SMatrix{4, 4, T}

export idmat
idmat(T::Type{<:SMatrix}) = T(I)


# ===== (Abstract) Types =====

export AbstractScene, Scene3D
"""`Scene{E, T}`
 Represents the entire scene. Privided primarily for convenience. It is intended to support cross-dimensional entities
 as well, e.g. 2D entities in a 3D scene. The implementation specifics of the scene are left to the implementer."""
abstract type AbstractScene{E, T<:Real} end

struct Scene3D{E, T<:Real} <: AbstractScene{E, T}
    roots::Set{E}
end
Scene3D{E, T}() where {E, T<:Real} = Scene3D{E, T}(Set{E}())


export Transform
"""`Transform{D, E, T}`
 Represents the composable transformation matrix of a scene node / entity.
 
 `D` is the transform's dimensionality.
 
 `E` is the type of the actual scene node. This type must specialize `SceneGraphs.transformof(node::E)`.
 
 `T` is the underlying numeric type of the transform."""
abstract type Transform{D, E, T<:Real} end

export SpriteTransform
"""`SpriteTransform` is a transform of 2D entities in a 3D scene."""
mutable struct SpriteTransform{E, T<:Real} <: Transform{3, E, T}
    scene::Optional{AbstractScene{E, T}}
    parent::Optional{E}
    children::Vector{E}
    location::Vector3{T}
    rotation::T
    scale::Vector2{T}
    dirty::Bool
    obj2world::Matrix4{T}
    world2obj::Matrix4{T}
end
function SpriteTransform{E, T}(location::Vector3 = Vector3{T}(0, 0, 0), rotation::Number = 0, scale::Vector2 = Vector2{T}(1, 1)) where {E, T}
    SpriteTransform{E, T}(nothing, nothing, Vector(), Vector3{T}(location...), T(rotation), Vector2{T}(scale...), true, idmat(Matrix4{T}), idmat(Matrix4{T}))
end

@generate_properties SpriteTransform begin
    @set location = setlocation!(self, value)
    @set rotation = setrotation!(self, value)
    @set scale    = setscale!(self, value)
end


# ===== Scene Alteration =====

Base.:(∈)(entity::E, scene::AbstractScene{E}) where E = entity ∈ scene.roots
Base.haskey(scene::AbstractScene{E}, entity::E) where E = entity ∈ scene
function Base.push!(scene::AbstractScene{E}, entity::E) where E
    tf = transformof(entity)
    if parentof(tf) !== nothing
        deparent!(tf)
    end
    update_scene!(tf, scene)
    push!(scene.roots, entity)
    scene
end
function Base.delete!(scene::AbstractScene{E}, entity::E) where E
    tf = transformof(entity)
    update_scene!(tf, nothing)
    delete!(scene.roots, entity)
    scene
end


# ===== Matrix Component Setters =====

export setlocation!
setlocation!(tf::Transform, it) = setlocation!(tf, it...)
function setlocation!(tf::Transform{D}, comps::Real...) where D
    setfield!(tf, :dirty, true)
    setfield!(tf, :location, compose_vector(D, tf.location, comps))
    return tf
end

export setrotation!
setrotation!(tf::Transform, it) = setrotation!(tf, it...)
function setrotation!(tf::Transform{D}, comps::Real...) where D
    setfield!(tf, :dirty, true)
    setfield!(tf, :rotation, compose_vector(D, tf.rotation, comps))
    return tf
end
function setrotation!(tf::SpriteTransform, rot::Real)
    setfield!(tf, :dirty, true)
    setfield!(tf, :rotation, rot)
    return tf
end

export setscale!
setscale!(tf::Transform, it) = setscale!(tf, it...)
setscale!(tf::Transform{D}, s::Real) where D = setscale!(tf, (s for _ ∈ 1:D)...)
function setscale!(tf::Transform{D}, comps::Real...) where D
    setfield!(tf, :dirty, true)
    setfield!(tf, :scale, compose_vector(D, tf.scale, comps))
    return tf
end
function setscale!(tf::SpriteTransform{E, T}, scalex::Real, scaley::Real) where {E, T<:Real}
    setfield!(tf, :dirty, true)
    setfield!(tf, :scale, Vector2{T}(scalex, scaley))
    return tf
end


# ===== Matrix Component Getters (object-local & scene-global) =====

export locationof
locationof(entity) = locationof(transformof(entity))
locationof(tf::Transform) = tf.location

export rotationof
rotationof(entity) = rotationof(transformof(entity))
rotationof(tf::Transform) = tf.rotation

export scaleof
scaleof(entity) = scaleof(transformof(entity))
scaleof(tf::Transform) = tf.scale

export worldlocationof
worldlocationof(entity) = worldlocationof(transformof(entity))
worldlocationof(tf::Transform{D}) where D = (mat = obj2world(tf); mat[1:D, lastindex(mat, 2)])

export worldrotationof
worldrotationof(entity) = worldrotationof(transformof(entity))
function worldrotationof(tf::Transform)
    final_rotation = rotationof(tf)
    curr = parentof(tf)
    while curr !== nothing
        final_rotation = final_rotation .+ rotationof(curr)
        curr = parentof(curr)
    end
    return final_rotation
end

export worldscaleof
worldscaleof(entity) = worldscaleof(transformof(entity))
function worldscaleof(tf::Transform)
    final_scale = scaleof(tf)
    curr = parentof(tf)
    while curr !== nothing
        final_scale = final_scale .* scaleof(curr)
        curr = parentof(curr)
    end
    return final_scale
end


# ===== Matrix Component Modifiers =====

export transform!
function transform!(entity; translate = nothing, rotate = nothing, scale = nothing)
    tf = transformof(entity)
    if translate !== nothing
        translate!(tf, translate...)
    end
    if rotate !== nothing
        rotate!(tf, rotate...)
    end
    if scale !== nothing
        scale!(tf, scale...)
    end
    entity
end

export translate!
translate!(tf::Transform, offset) = translate!(tf, offset...)
translate!(tf::Transform{D}, coords::Real...) where D = translate!(tf, compose_vector(D, zeros(D), coords)...)
translate!(tf::Transform{D}, args::Vararg{Real, D}) where D = (tf.location = tf.location .+ args; tf)
translate!(entity, args...) = (translate!(transformof(entity), args...); entity)

export scale!
scale!(tf::Transform, scale) = scale!(tf, scale...)
scale!(tf::Transform{D}, scale::Real...) where D = scale!(tf, compose_vector(D, ones(D), scale)...)
scale!(tf::Transform{D}, scale::Vararg{Real, D}) where D = tf.scale = tf.scale .* scale
scale!(entity, args...) = (scale!(transformof(entity), args...); entity)

scale!(tf::SpriteTransform, scalex::Real, scaley::Real) = tf.scale = tf.scale .* (scalex, scaley)

export rotate!
rotate!(tf::Transform, offset) = rotate!(tf, offset...)
rotate!(tf::Transform{D}, angles::Real...) where D = rotate!(tf, compose_vector(D, zeros(D), angles)...)
rotate!(tf::Transform{D}, angles::Vararg{Real, D}) where D = tf.rotation = tf.rotation .+ angles
rotate!(entity, args...) = (rotate!(transformof(entity), args...); entity)

# Specializations
rotate!(tf::SpriteTransform, rotation::Real) = tf.rotation += rotation


# ===== Scene Graph Alteration =====

export parent!
"""`parent!(child, parent)`
 Assign `child` to its new `parent`. This automatically marks the child as *dirty*. Both `child` and `parent` are
 assumed to have transforms."""
parent!(::Nothing, _) = throw(ArgumentError("child is nothing"))
parent!(_, ::Nothing) = throw(ArgumentError("parent is nothing"))
parent!(child, parent) = (parent!(transformof(child), transformof(parent), child, parent); child)

function parent!(tf_child::Transform, tf_parent::Transform, child, parent)
    if child === parent || tf_child === tf_parent
        throw(ArgumentError("cannot parent an entity to itself"))
    end
    
    if parentof(tf_child) !== parent
        flagdirty!(child)
        
        deparent!(tf_child, child)
        push!(tf_parent.children, child)
        tf_child.parent = parent
        update_scene!(tf_child, tf_parent.scene)
    end
    return
end

export deparent!
deparent!(child) = deparent!(transformof(child), child)
deparent!(_, ::Transform) = throw(ArgumentError("missing child entity, only got transform"))
function deparent!(tf::Transform, child)
    if parentof(child) !== nothing
        tf_parent = child |> parentof |> transformof
        if (i = indexof(tf_parent.children, child)) !== nothing
            deleteat!(tf_parent.children, i)
        end
        
        tf.parent = nothing
        tf.dirty  = true
        update_scene!(tf, nothing)
    end
    
    return child
end

function update_scene!(tf::Transform, scene::Union{Nothing, AbstractScene})
    tf.dirty = true
    tf.scene = scene
    for child ∈ childrenof(tf)
        update_scene!(transformof(child), scene)
    end
    return tf
end


# ===== Trait Interface =====

export sceneof, parentof, childrenof
sceneof(x)    = transformof(x).scene
parentof(x)   = transformof(x).parent
childrenof(x) = transformof(x).children
childrenof(T::Type, x) = filter(c->isa(c, T), childrenof(x))

function ExtraFun.update!(tf::Transform{3, E, T}, parentmat::Matrix4{T} = idmat(Matrix4{T}), forceupdate::Bool = false) where {E, T}
    if tf.dirty || forceupdate
        tf.obj2world = parentmat * transformmatrix4(tf)
        tf.world2obj = inv(tf.obj2world)
        
        forceupdate = true
        cleardirty!(tf)
    end
    
    @threads for child ∈ childrenof(tf)
        update!(transformof(child), tf.obj2world, forceupdate)
    end
    
    return tf
end

export flagdirty!, cleardirty!
flagdirty!(entity) = (flagdirty!(transformof(entity)); entity)
flagdirty!(tf::Transform) = (tf.dirty = true; tf)
cleardirty!(entity) = (cleardirty!(transformof(entity)); entity)
cleardirty!(tf::Transform) = (tf.dirty = false; tf)

export obj2world, world2obj
obj2world(x) = obj2world(transformof(x))
world2obj(x) = world2obj(transformof(x))
obj2world(transform::Transform) = transform.obj2world
world2obj(transform::Transform) = transform.world2obj

export hastransform
@generated function hastransform(x)
    if hassignature(transformof, x)
        :(true)
    else
        :(false)
    end
end

export transformof, transform_family, transform_dimensionality, transform_entity_type, transform_number_type
transformof(x)::Transform = x.transform
transformof(x::Transform) = x
transform_family(T::Type{<:Transform}) = throw(MethodError(transform_family, (T,)))
transform_family(::Type{<:SpriteTransform}) = SpriteTransform
transform_dimensionality(::Type{<:Transform{D, E, T}}) where {D, E, T} = D
transform_entity_type(::Type{<:Transform{D, E, T}}) where {D, E, T} = E
transform_number_type(::Type{<:Transform{D, E, T}}) where {D, E, T} = T


# ===== Transformation (Matrix) Methods =====

export translationmatrix4, rotationmatrix4, scalematrix4
translationmatrix4(T::Type{<:Real}, x::Real, y::Real, z::Real) = Matrix4{T}(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1)
rotationmatrix4(T::Type{<:Real}, p::Real, y::Real, r::Real) = yawmatrix(T, y) * pitchmatrix(T, p) * rollmatrix(T, r)
scalematrix4(T::Type{<:Real}, x::Real, y::Real, z::Real = 1) = Matrix4{T}(x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1)
scalematrix4(T::Type{<:Real}, s::Real) = scalematrix4(T, s, s, s)

export pitchmatrix, yawmatrix, rollmatrix
pitchmatrix(T::Type{<:Real}, p::Real) = (sinr = sin(p); cosr = cos(p); Matrix4{T}(1, 0, 0, 0, 0, cosr, sinr, 0, 0, -sinr, cosr, 0, 0, 0, 0, 1))
yawmatrix(  T::Type{<:Real}, y::Real) = (sinr = sin(y); cosr = cos(y); Matrix4{T}(cosr, 0, -sinr, 0, 0, 1, 0, 0, sinr, 0, cosr, 0, 0, 0, 0, 1))
rollmatrix( T::Type{<:Real}, r::Real) = (sinr = sin(r); cosr = cos(r); Matrix4{T}(cosr, sinr, 0, 0, -sinr, cosr, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1))

export transformmatrix4
transformmatrix4(tf::Transform{D, E, T}) where {D, E, T} = translationmatrix4(T, tf.location...) * rotationmatrix4(T, tf.rotation...) * scalematrix4(T, tf.scale...)
transformmatrix4(tf::SpriteTransform{E, T}) where {E, T} = translationmatrix4(T, tf.location...) * rollmatrix(T, tf.rotation) * scalematrix4(T, tf.scale...)

export transform
transform(tf, ::Nothing) = tf
transform(vert::SVector{D}, mat::SMatrix{D, D}) where D = mat * vert
function transform(verts, mat::SMatrix{N, N}) where N
    transformed = [vformat(vert, Vector4(0, 0, 0, 1)) for vert ∈ verts]
    @threads for i ∈ 1:length(transformed)
        transformed[i] = mat * vformat(transformed[i], Vector4(0, 0, 0, 1))
    end
    transformed
end


# === Base Overrides ===
Base.copy(tf::SpriteTransform{E, T}) where {E, T} = SpriteTransform{E, T}(nothing, nothing, [], tf.location, tf.rotation, tf.scale, true, idmat(Matrix4), idmat(Matrix4))

function Base.show(io::IO, transform::Transform)
    write(io, "$(typeof(transform))(loc: $(transform.location), rot: $(transform.rotation), scale: $(transform.scale)")
    if transform.dirty
        write(io, ", dirty")
    end
    write(io, ")")
end

# === Utilities ===
function compose_vector(D::Integer, missing_defaults, comps)
    if length(comps) < D
        SVector{D}(comps..., (missing_defaults[i] for i ∈ length(comps)+1:D)...)
    else
        SVector{D}((comps[i] for i ∈ 1:D)...)
    end
end

end # module SceneGraphs
