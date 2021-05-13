######################################################################
# Scene graph UTs
# -----
# Licensed under Apache License 2.0
using Test
using ExtraFun
using SceneGraphs
using StaticArrays

abstract type AbstractEntity end

const TestScene = Scene3D{AbstractEntity, Float64}

EntityTransform = SpriteTransform{AbstractEntity, Float64}

struct Entity <: AbstractEntity
    name::Symbol
    transform::EntityTransform
end
Entity(name::Symbol) = Entity(name, EntityTransform())

function transform_matrix(translation, rotation, scale)
    mt = @SMatrix([1 0 0 translation[1];
                   0 1 0 translation[2];
                   0 0 1 translation[3];
                   0 0 0 1])
    
    rcos = cos(deg2rad(rotation))
    rsin = sin(deg2rad(rotation))
    mr = @SMatrix([rcos -rsin 0 0;
                   rsin  rcos 0 0;
                   0     0    1 0;
                   0     0    0 1])
    
    ms = @SMatrix([scale[1] 0 0 0;
                   0 scale[2] 0 0;
                   0 0 1 0;
                   0 0 0 1])
    
    return mt*mr*ms
end


macro test_nothrow(expr)
    esc(:(@test begin $(expr); true end))
end

@testset "graph" begin
    let scene = TestScene(), entity1 = Entity(:first), entity2 = Entity(:second), entity3 = Entity(:third), entity4 = Entity(:fourth)
        push!(scene, entity1)
        push!(scene, entity2)
        @test entity1 ∈ scene && entity2 ∈ scene
        
        @test sceneof(entity1) === scene && sceneof(entity2) === scene && sceneof(entity3) === nothing
        
        @test parent!(entity4, entity3) === entity4
        @test parentof(entity4) === entity3 && sceneof(entity4) === nothing && sceneof(entity3) === sceneof(entity4)
        
        @test parent!(entity3, entity1) === entity3
        @test parentof(entity3) === entity1 && parentof(entity4) === entity3 && sceneof(entity3) === scene && sceneof(entity1) === sceneof(entity3) && sceneof(entity3) === sceneof(entity4)
        @test entity3 ∈ childrenof(entity1) && entity3 ∉ childrenof(entity2)
        
        @test parent!(entity3, entity2) === entity3
        @test parentof(entity3) === entity2 && parentof(entity4) === entity3 && sceneof(entity3) === scene && sceneof(entity2) === sceneof(entity3) && sceneof(entity3) === sceneof(entity4)
        @test entity3 ∉ childrenof(entity1) && entity3 ∈ childrenof(entity2)
        
        @test deparent!(entity3) === entity3
        @test parentof(entity3) === nothing && parentof(entity4) === entity3
        @test sceneof(entity3) === nothing && sceneof(entity4) === nothing
        @test entity3 ∉ childrenof(entity1) && entity3 ∉ childrenof(entity2)
    end
end

@testset "transformation" begin
    @testset "translate" begin
        let scene = TestScene(), parent = Entity(:parent), interim = Entity(:interim), child = Entity(:child)
            push!(scene, parent)
            
            parent!(child, interim)
            parent!(interim, parent)
            
            @test_nothrow translate!(parent, 1, 2, 3)
            @test_nothrow translate!(interim, 2, 3, 4)
            update!(transformof(parent))
            
            @test worldlocationof(parent)  ≈ [1, 2, 3]
            @test worldlocationof(interim) ≈ [3, 5, 7]
            @test worldlocationof(child)   ≈ [3, 5, 7]
        end
    end
    
    @testset "rotate" begin
        let scene = TestScene(), parent = Entity(:parent), interim = Entity(:interim), child = Entity(:child)
            push!(scene, parent)
            
            parent!(child, interim)
            parent!(interim, parent)
            
            @test_nothrow rotate!(parent, deg2rad(45))
            @test_nothrow rotate!(interim, deg2rad(30))
            update!(transformof(parent))
            
            @test worldrotationof(parent)  ≈ deg2rad(45)
            @test worldrotationof(interim) ≈ deg2rad(75)
            @test worldrotationof(child)   ≈ deg2rad(75)
        end
    end
    
    @testset "scale" begin
        let scene = TestScene(), parent = Entity(:parent), interim = Entity(:interim), child = Entity(:child)
            push!(scene, parent)
            
            parent!(child, interim)
            parent!(interim, parent)
            
            @test_nothrow scale!(parent, 1, 2)
            @test_nothrow scale!(interim, 4, 2)
            update!(transformof(parent))
            
            @test worldscaleof(parent)  ≈ [1, 2]
            @test worldscaleof(interim) ≈ [4, 4]
            @test worldscaleof(child)   ≈ [4, 4]
        end
    end
    
    @testset "combined" begin
        let scene = TestScene(), parent = Entity(:parent), interim = Entity(:interim), child = Entity(:child)
            push!(scene, parent)
            
            parent!(child, interim)
            parent!(interim, parent)
            
            transform!(parent,  translate=(1, 2, 3), rotate=deg2rad(45), scale=(1, 2))
            transform!(interim, translate=(2, 3, 4), rotate=deg2rad(30), scale=(4, 2))
            update!(transformof(parent))
            
            r45 = deg2rad(45)
            r30 = deg2rad(30)
            m_p = transform_matrix((1, 2, 3), 45, (1, 2))
            m_i = transform_matrix((2, 3, 4), 30, (4, 2))
            m_c = transform_matrix((0, 0, 0),  0, (1, 1))
            
            @test obj2world(parent)  ≈ m_p
            @test obj2world(interim) ≈ m_p*m_i
            @test obj2world(child)   ≈ m_p*m_i*m_c
        end
    end
end
