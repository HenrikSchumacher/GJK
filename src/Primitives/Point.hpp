#pragma once


#define CLASS Point
#define BASE  PrimitiveSerialized<AMB_DIM,Real,Int,SReal>

namespace GJK
{
    
    template<int AMB_DIM,typename Real,typename Int,typename SReal,
                typename ExtReal = SReal,typename ExtInt = Int>
    class CLASS : public BASE
    {
        ASSERT_FLOAT (ExtReal );
        ASSERT_INT   (ExtInt  );
        
    protected:
        
        // serialized_data is assumed to be an array of size SIZE. Will never be allocated by class! Instead, it is meant to be mapped onto an array of type Real by calling the member SetPointer.
        
        // DATA LAYOUT
        // serialized_data[0] = squared radius = 0
        // serialized_data[1],...,serialized_data[AMB_DIM] = interior_point
        
    public:
        
        CLASS() : BASE() {}
        
        // Copy constructor
        CLASS( const CLASS & other ) : BASE( other ) {}

        // Move constructor
        CLASS( CLASS && other ) noexcept : BASE( other ) {}

        virtual ~CLASS() override = default;
        
        static constexpr Int SIZE = 1 + AMB_DIM;
        
        virtual constexpr Int Size() const override
        {
            return SIZE;
        }

    protected:

        mutable SReal self_buffer [SIZE];
        
#include "Primitive_BoilerPlate.hpp"
        
        __ADD_CLONE_CODE__(CLASS)
        
    public:
        
        virtual void FromCoordinates( const ExtReal * const coords, const Int i = 0 ) const
        {
            serialized_data[0] = static_cast<SReal>(0);
            
            for( Int k = 0; k < AMB_DIM; ++k )
            {
                serialized_data[k+1] = static_cast<SReal>( coords[ AMB_DIM * i + k ] );
            }
        }
        
        
        //Computes support vector supp of dir.
        virtual Real MinSupportVector( ptr<Real> v, mut<Real> s ) const override
        {
            ptr<SReal> c = &serialized_data[1];

            Real value = static_cast<Real>(c[0]) * v[0];
            
            s[0] = c[0];
            
            for( Int k = 1; k < AMB_DIM; ++k )
            {
                value += static_cast<Real>(c[k]) * v[k];
                s[k] = c[k];
            }

            return value;
        }
        
        //Computes support vector supp of dir.
        virtual Real MaxSupportVector( ptr<Real> v, mut<Real> s ) const override
        {
            return MinSupportVector( v, s );
        }

        // Computes only the values of min/max support function. Usefull to compute bounding boxes.
        virtual void MinMaxSupportValue( ptr<Real> v, Real & min_val, Real & max_val ) const override
        {
            ptr<SReal> center = &serialized_data[1];

            Real value = static_cast<Real>(center[0]) * v[0];
            
            for( Int k = 1; k < AMB_DIM; ++k )
            {
                value += static_cast<Real>(center[k]) * v[k];
            }
            
            min_val = value;
            max_val = value;
        }
        
        
        // Helper function to compute axis-aligned bounding boxes. in the format of box_min, box_max vector.
        // box_min, box_max are supposed to be vectors of size AMB_DIM.
        // BoxMinMax computes the "lower left" lo and "upper right" hi vectors of the primitives bounding box and sets box_min = min(lo, box_min) and box_max = min(h, box_max)
        virtual void BoxMinMax( SReal * const box_min_, SReal * const box_max_ ) const
        {
            copy_buffer<AMB_DIM>( &serialized_data[1], box_min_ );
            copy_buffer<AMB_DIM>( &serialized_data[1], box_max_ );
        }
        
        
        virtual std::string ClassName() const override
        {
            return TO_STD_STRING(CLASS)+"<"+ToString(AMB_DIM)+","+TypeName<Real>+","+TypeName<Int>+","+TypeName<SReal>+","+TypeName<ExtReal>+","+TypeName<ExtInt>+">";
        }

    }; // Point
    
} // namespe GJK

#undef CLASS
#undef BASE
