#pragma once

#define CLASS AABB
#define BASE  BoundingVolumeBase<AMB_DIM,Real,Int,SReal>

namespace GJK
{
    using namespace Tensors;
    
    // serialized_data is assumed to be an array of size SIZE. Will never be allocated by class! Instead, it is meant to be mapped onto an array of type SReal by calling the member SetPointer.
    
    // DATA LAYOUT
    // serialized_data[0] = squared radius
    // serialized_data[1],...,serialized_data[AMB_DIM] = center
    // serialized_data[AMB_DIM + 1],...,serialized_data[AMB_DIM + AMB_DIM] = half the edge lengths.
    
    template<int AMB_DIM, typename Real, typename Int, typename SReal>
    class CLASS : public BASE
    {
    protected:
        
        using Vector_T = Tiny::Vector<AMB_DIM,        SReal,Int>;
        using Matrix_T = Tiny::Matrix<AMB_DIM,AMB_DIM,SReal,Int>;
        
        Tiny::Matrix<AMB_DIM,AMB_DIM,Real,Int> id_matrix;
    
    public:
        
        CLASS() : BASE()
        {
            id_matrix.SetIdentity();
        }

        // Copy constructor
        CLASS( const CLASS & other ) : BASE( other )
        {
            id_matrix.SetIdentity();
        }
        
        // Move constructor
        CLASS( CLASS && other ) noexcept  : BASE( other )
        {
            id_matrix.SetIdentity();
        }
        
        virtual ~CLASS() override = default;
        
        
        static constexpr Int SIZE = 1 + AMB_DIM + AMB_DIM;
        
        virtual constexpr Int Size() const override
        {
            return SIZE;
        }
        
    protected:

        mutable SReal r2;
        
        mutable Vector_T lower;
        mutable Vector_T upper;
        
//        mutable SReal self_buffer [1 + AMB_DIM + AMB_DIM];
 
    public:
        
#include "../Primitives/Primitive_Common.hpp"
        
        __ADD_CLONE_CODE_FOR_ABSTRACT_CLASS__(CLASS)
        
        
    public:
        
        virtual void Load( mptr<SReal> p_, const Int pos ) override
        {
            SetPointer( p_, pos );
            
            r2 = serialized_data[0];
            lower.Read( &serialized_data[1]         );
            upper.Read( &serialized_data[1+AMB_DIM] );
        }

        virtual void Load( mptr<SReal> p_ ) override
        {
            SetPointer( p_ );
            
            r2 = serialized_data[0];
            lower.Read( &serialized_data[1]         );
            upper.Read( &serialized_data[1+AMB_DIM] );
        }
        
        
        // array p is suppose to represent a matrix of size N x AMB_DIM
        void FromPointCloud( cptr<SReal> coords_in, const Int N ) const override
        {
//            tic(ClassName()+"::FromPointCloud");
            
            Vector_T x ( &coords_in[ AMB_DIM * 0] );
            
            lower = x;
            upper = x;
            
            for( Int i = 1; i < N; ++i )
            {
                x.Read( &coords_in[ AMB_DIM * i] );
                
                for( Int k = 0; k < AMB_DIM; ++k )
                {
                    lower[k] = Min( lower[k], x[k] );
                    upper[k] = Max( upper[k], x[k] );
                }
            }
            
            lower.Write( &serialized_data[1]);
            upper.Write( &serialized_data[1 + AMB_DIM]);
            
            x  = upper;
            x -= lower;
            
            serialized_data[0] = r2 = Scalar::Quarter<SReal> * x.SquaredNorm();
            
            
//            toc(ClassName()+"::FromPointCloud");
        }
        

        // array p is supposed to represent a matrix of size N x AMB_DIM
        void FromPrimitives(
            mref<PolytopeBase<AMB_DIM,Real,Int,SReal>> P,  // primitive prototype
            mptr<SReal> P_serialized,                      // serialized data of primitives
            const Int begin,                               // which _P_rimitives are in question
            const Int end,                                 // which _P_rimitives are in question
            Int thread_count = 1                           // how many threads to utilize
        ) const
        {
//            ptic(ClassName()+"::FromPrimitives (PolytopeBase)");

            lower.Fill( std::numeric_limits<SReal>::max()    );
            upper.Fill( std::numeric_limits<SReal>::lowest() );
            
            // TODO: Parallelize this!
            for( Int i = begin; i < end; ++i )
            {
                P.SetPointer( P_serialized, i );
                P.BoxMinMax( lower.data(), upper.data() );
            }

            lower.Write( &serialized_data[1          ]);
            upper.Write( &serialized_data[1 + AMB_DIM]);
            
            Vector_T x;
            
            x  = upper;
            x -= lower;
            
            serialized_data[0] = r2 = Scalar::Quarter<SReal> * x.SquaredNorm();
            
//            ptoc(ClassName()+"::FromPrimitives (PolytopeBase)");
        }
        
        // array p is supposed to represent a matrix of size N x AMB_DIM
        virtual void FromPrimitives(
            mref<PrimitiveSerialized<AMB_DIM,Real,Int,SReal>> P,      // primitive prototype
            mptr<SReal> P_serialized,                  // serialized data of primitives
            const Int begin,                           // which _P_rimitives are in question
            const Int end,                             // which _P_rimitives are in question
            Int thread_count = 1                       // how many threads to utilize
        ) const override
        {
//            ptic(ClassName()+"::FromPrimitives (PrimitiveSerialized)");
        
            lower.Fill( std::numeric_limits<SReal>::max()    );
            upper.Fill( std::numeric_limits<SReal>::lowest() );
            
            // TODO: Parallelize this!
            for( Int i = begin; i < end; ++i )
            {
                P.SetPointer( P_serialized, i );
                
                for( Int j = 0; j < AMB_DIM; ++j )
                {
                    Real min_val;
                    Real max_val;
                    
                    P.MinMaxSupportValue( &id_matrix[j][0], min_val, max_val );
                    lower[j] = Min( lower[j], static_cast<SReal>(min_val) );
                    upper[j] = Max( upper[j], static_cast<SReal>(max_val) );
                }
            }
            
            lower.Write( &serialized_data[1          ]);
            upper.Write( &serialized_data[1 + AMB_DIM]);
            
            Vector_T x;
            
            x  = upper;
            x -= lower;
            
            serialized_data[0] = r2 = Scalar::Quarter<SReal> * x.SquaredNorm();
//            ptoc(ClassName()+"::FromPrimitives (PrimitiveSerialized)");
        }
        
        
        
        // Returns some point within the primitive and writes it to p.
        void InteriorPoint( mptr<Real> point_out ) const override
        {
//            print(ClassName()+"::InteriorPoint");
            if( this->serialized_data != nullptr )
            {
                Vector_T x ( lower );
                
                x += upper;
                
                x *= Scalar::Half<Real>;
                
                x.Write( point_out );
            }
            else
            {
                print("this->serialized_data = " + ToString(this->serialized_data) );
                valprint("this->serialized_data[1]",this->serialized_data[1]);
                valprint("this->serialized_data[2]",this->serialized_data[2]);
                valprint("this->serialized_data[3]",this->serialized_data[3]);
                eprint(ClassName()+"::InteriorPoint: serialized_data is set to nullptr. Doing nothing.");
            }
        }
        
        virtual Real InteriorPoint( const Int k ) const override
        {
            if( this->serialized_data )
            {
                return static_cast<Real>(
                    Scalar::Half<SReal> * ( lower[k] + upper[k] )
                );
            }
            else
            {
//                valprint("this->serialized_data",this->serialized_data);
//                valprint("this->serialized_data[1+k]",this->serialized_data[1+k]);
                eprint(ClassName()+"::InteriorPoint: serialized_data is set to nullptr. Return 0.");
                return Scalar::Zero<Real>;
            }
        }
        
        
        
        
        //Computes support vector supp of dir.
        virtual Real MaxSupportVector( cptr<Real> dir, mptr<Real> supp ) const override
        {
            Vector_T v ( dir );
            Vector_T s;
            
            SReal result = Scalar::Zero<SReal>;
            
            for( Int k = 0; k < AMB_DIM; ++k )
            {
                const SReal a = v[k] * lower[k];
                const SReal b = v[k] * upper[k];
                
                if( a < b )
                {
                    result += b;
                    s[k] = upper[k];
                }
                else
                {
                    result += a;
                    s[k] = lower[k];
                }
                
            }

            s.Write(supp);
            
            return static_cast<Real>(result);
        }


        //Computes support vector supp of dir.
        virtual Real MinSupportVector( cptr<Real> dir, mptr<Real> supp ) const override
        {
            Vector_T v ( dir );
            Vector_T s;
            
            SReal result = Scalar::Zero<SReal>;
            
            for( Int k = 0; k < AMB_DIM; ++k )
            {
                const SReal a = v[k] * lower[k];
                const SReal b = v[k] * upper[k];
                
                if( a > b )
                {
                    result += b;
                    s[k] = upper[k];
                }
                else
                {
                    result += a;
                    s[k] = lower[k];
                }
                
            }

            s.Write(supp);
            
            return static_cast<Real>(result);
        }
                
        
        virtual void MinMaxSupportValue( cptr<Real> dir, mref<Real> min_val, mref<Real> max_val ) const override
        {
            Vector_T v ( dir );
            
            SReal min_val_ = Scalar::Zero<Real>;
            SReal max_val_ = Scalar::Zero<Real>;
            
            for( Int k = 0; k < AMB_DIM; ++k )
            {
                const SReal a = v[k] * lower[k];
                const SReal b = v[k] * upper[k];
                
                min_val += Min(a,b);
                max_val += Max(a,b);
                
            }
            
            min_val_ = static_cast<Real>(min_val);
            max_val_ = static_cast<Real>(max_val);
        }
        
        
        virtual std::string ClassName() const override
        {
            return TO_STD_STRING(CLASS)+"<"+ToString(AMB_DIM)+","+TypeName<Real>+","+TypeName<Int>+","+TypeName<SReal>+">";
        }
        
        
        inline friend Real AABB_SquaredDistance( cref<CLASS> P, cref<CLASS> Q )
        {
            SReal d2 = Scalar::Zero<Real>;
            
            for( Int k = 0; k < AMB_DIM; ++k )
            {
                SReal x = static_cast<Real>(
                    Ramp( Max( P.lower[k], Q.lower[k] ) - Min( P.upper[k], Q.upper[k] ) )
                );
                d2 += x * x;
            }
            return static_cast<Real>(d2);
        }
        
        void Merge( mptr<SReal> C_Serialized, const Int i = 0 ) const
        {
            if( C_Serialized != nullptr )
            {
                Vector_T Q_lower ( &C_Serialized[SIZE * i + 1          ] );
                Vector_T Q_upper ( &C_Serialized[SIZE * i + 1 + AMB_DIM] );
                

                for( Int k = 0; k < AMB_DIM; ++k )
                {
                    lower[k] = Min( lower[k], Q_lower[k] );
                    upper[k] = Max( upper[k], Q_upper[k] );
                    
                    r2 += (upper[k] - lower[k]) * (upper[k] - lower[k]);
                }
                
                Vector_T x ( upper );
                
                x -= lower;
                
                serialized_data[0] = Scalar::Quarter<SReal> * x.SquaredNorm();
            }
        }
        
    }; // CLASS

} // namespace GJK

#undef CLASS
#undef BASE
    

