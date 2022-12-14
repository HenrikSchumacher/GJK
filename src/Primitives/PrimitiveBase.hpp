#pragma once

namespace GJK
{
    
#define CLASS PrimitiveBase
    
    // Real  -  data type that will be handed to GJK; GJK typically needs doubles.
    // Int   -  integer type for return values and loops.
    
    template<int AMB_DIM, typename Real_, typename Int_>
    class alignas( OBJECT_ALIGNMENT ) CLASS // Use this broad alignment to prevent false sharing.
    {
        ASSERT_FLOAT(Real_);
        ASSERT_INT  (Int_ );

    public:
        
        using Real = Real_;
        using Int  = Int_;
        
        CLASS() = default;
        
        virtual ~CLASS() = default;
        
        __ADD_CLONE_CODE_FOR_BASE_CLASS__(CLASS)
        
    public:
        
        static constexpr Int AmbDim()
        {
            return AMB_DIM;
        }
        
        // Computes support vector supp of dir.
        virtual Real MaxSupportVector( const Real * const dir, Real * const supp ) const = 0;
        
        // Computes support vector supp of dir.
        virtual Real MinSupportVector( const Real * const dir, Real * const supp ) const = 0;
        
        // Computes only the values of min/max support function. Usefull to compute bounding boxes.
        virtual void MinMaxSupportValue( const Real * const dir, Real & min_val, Real & max_val ) const = 0;
        
        // Returns some point within the primitive and writes it to p.
        virtual void InteriorPoint( Real * const p ) const = 0;
    
        virtual Real InteriorPoint( const Int k ) const = 0;
        
        // Returns some (upper bound of the) squared radius of the primitive as measured from the result of InteriorPoint.
        virtual Real SquaredRadius() const = 0;

        virtual std::string DataString() const = 0;
        
        virtual std::string ClassName() const
        {
            return "PrimitiveBase<"+TypeName<Real>::Get()+","+ToString(AMB_DIM)+">";
        }
        
    }; // PrimitiveBase

#undef CLASS
    
} // namespace GJK


