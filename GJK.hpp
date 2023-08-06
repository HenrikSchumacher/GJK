#ifndef GJK_HPP

    #define GJK_HPP

    #include "submodules/Tools/Tools.hpp"

    #define GJK_STRINGIFY(a) #a

    namespace GJK
    {
        using GJK_Real = double;
        
        using namespace Tools;
        
    } // namespace Collision


    #ifdef GJK_Report

        #define GJK_DUMP(x) DUMP(x);

        #define GJK_print(x) logprint(x);

        #define GJK_tic(x) ptic(x);

        #define GJK_toc(x) ptoc(x);

    #else

        #define GJK_DUMP(x)

        #define GJK_tic(x)

        #define GJK_toc(x)

        #define GJK_print(x)

    #endif

    //#include "src/MortonCode.hpp"
    //#include "src/Collision/HilbertCurve.hpp"

    // These primitives are not serializable -- for a reason.
    #include "src/Primitives/PrimitiveBase.hpp"
    #include "src/Primitives/ConvexHull.hpp"

    // Serializable primitives:
    #include "src/Primitives/PrimitiveSerialized.hpp"
    #include "src/Primitives/Point.hpp"
    #include "src/Primitives/PolytopeBase.hpp"
    #include "src/Primitives/PolytopeExt.hpp"
    #include "src/Primitives/Polytope.hpp"
    #include "src/Primitives/Ellipsoid.hpp"
    #include "src/Primitives/Parallelepiped.hpp"

    //#include "src/Primitives/SpaceTimePrism.hpp"

    #include "src/Primitives/TypeDefs.hpp"

    // Bounding volume types:
    #include "src/BoundingVolumes/BoundingVolumeBase.hpp"
    #include "src/BoundingVolumes/AABB.hpp"
    #include "src/BoundingVolumes/AABB_LongestAxisSplit.hpp"
    #include "src/BoundingVolumes/AABB_MedianSplit.hpp"
    #include "src/BoundingVolumes/AABB_PreorderedSplit.hpp"
    //#include "src/BoundingVolumes/OBB.hpp"                    // requires eigensolve
    //#include "src/BoundingVolumes/OBB_MedianSplit.hpp"        // requires eigensolve
    //#include "src/BoundingVolumes/OBB_PreorderedSplit.hpp"    // requires eigensolve


    // Actual GJK algorithms
    #include "src/GJK_Algorithm.hpp"
    #include "src/GJK_Batch.hpp"
    #include "src/GJK_Offset_Batch.hpp"


    #include "src/Primitives/MovingPolytopeBase.hpp"
    #include "src/Primitives/MovingPolytopeExt.hpp"
    #include "src/Primitives/MovingPolytope.hpp"

    #include "src/CollisionFinderBase.hpp"
    #include "src/CollisionFinder.hpp"

#endif
