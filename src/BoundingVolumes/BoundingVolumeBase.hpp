#pragma once

#define CLASS BoundingVolumeBase
#define BASE  PrimitiveSerialized<AMB_DIM,Real,Int,SReal>

namespace GJK
{
    
    // Class that adds bounding volume features to a basic serializable primitive, like computing the bounding volume from a set of points.
    template<int AMB_DIM, typename Real, typename Int, typename SReal>
    class CLASS : public BASE
    {
    public:
        
        CLASS() : BASE()
        {}
        
        // Copy constructor
        CLASS( const CLASS & other ) : BASE(other)
        {}
        
        // Move constructor
        CLASS( CLASS && other ) noexcept : BASE(other)
        {}
        
        virtual ~CLASS() override = default;
        
//        virtual CLASS * Clone() const override = 0;
        
        __ADD_CLONE_CODE_FOR_ABSTRACT_CLASS__(CLASS)
        
    public:

        virtual void Load( mptr<SReal> p_ )
        {
            this->SetPointer( p_ );
        }

        virtual void Load( mptr<SReal> p_, const Int pos )
        {
            this->SetPointer( p_, pos );
        }
        
        // array p is suppose to represent a matrix of size N x AMB_DIM
        virtual void FromPointCloud( cptr<SReal> coords, const Int N ) const = 0;
        
        virtual void FromPrimitives(
            PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & P,
            mptr<SReal> P_data,
            const Int begin,
            const Int end,           // which _P_primitives are in question
            Int thread_count = 1     // how many threads to utilize
        ) const = 0;
        
        virtual Int Split(
            PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & P,                    // primitive prototype; to be "mapped" over P_data, thus not const.
            mptr<SReal> P_data, const Int begin, const Int end,    // which _P_rimitives are in question
            mptr<Int>   P_ordering,                                // to keep track of the permutation of the primitives
            mptr<SReal> C_data, const Int C_ID,                    // where to get   the bounding volume info for _C_urrent bounding volume
            mptr<SReal> L_data, const Int L_ID,                    // where to store the bounding volume info for _L_eft  child (if successful!)
            mptr<SReal> R_data, const Int R_ID,                    // where to store the bounding volume info for _R_ight child (if successful!)
            mptr<SReal> score,                                     // some scratch buffer for one scalar per primitive
            mptr<Int>   perm,                                      // some scratch buffer for one Int per primitive (for storing local permutation)
            mptr<Int>   inv_perm,                                  // some scratch buffer for one Int per primitive (for storing inverse of local permutation)
            Int thread_count = 1                                   // how many threads to utilize
        ) = 0;

        virtual std::string ClassName() const override
        {
            return TO_STD_STRING(CLASS)+"<"+ToString(AMB_DIM)+","+TypeName<Real>+","+TypeName<Int>+","+TypeName<SReal>+">";
        }
        
    }; // PrimitiveBase

    
} // namespace GJK

#undef BASE
#undef CLASS
