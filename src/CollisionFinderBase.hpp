#pragma once

#define CLASS CollisionFinderBase

namespace GJK
{
    
    template <typename Real, typename Int, typename SReal>
    class alignas( OBJECT_ALIGNMENT ) CLASS
    {
        ASSERT_FLOAT(Real);
        ASSERT_INT  (Int     );
        ASSERT_FLOAT(SReal   );
        
    public:
        
        CLASS() {};
        
        virtual ~CLASS() = default;

    protected:

    public:
        
        virtual SReal FindMaximumSafeStepSize(
            const SReal * p, const SReal * u,
            const SReal * q, const SReal * v,
            const SReal tinit
        ) const = 0;
        
    public:
        
        std::string ClassName() const
        {
            return TO_STD_STRING(CLASS)+"<"+TypeName<Real>+","+TypeName<Int>+","+TypeName<SReal>+","+">";
        }
    };
    
    
} // namespace GJK

#undef CLASS
