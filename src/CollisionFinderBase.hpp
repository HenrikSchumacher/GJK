#pragma once

#define CLASS CollisionFinderBase

namespace GJK
{
    
    template <typename Real, typename Int, typename SReal, typename ExtReal>
    class alignas( OBJECT_ALIGNMENT ) CLASS
    {
        ASSERT_FLOAT(Real);
        ASSERT_INT  (Int     );
        ASSERT_FLOAT(SReal   );
        ASSERT_FLOAT(ExtReal );
        
    public:
        
        CLASS() {};
        
        virtual ~CLASS() = default;

    protected:

    public:
        
        virtual ExtReal FindMaximumSafeStepSize(
            const SReal * p, const SReal * u,
            const SReal * q, const SReal * v,
            const SReal tinit
        ) const = 0;
        
    public:
        
        std::string ClassName() const
        {
            return TO_STD_STRING(CLASS)+"<"+TypeName<Real>::Get()+","+TypeName<Int>::Get()+","+TypeName<SReal>::Get()+","+TypeName<ExtReal>::Get()+","+">";
        }
    };
    
    
} // namespace GJK

#undef CLASS
