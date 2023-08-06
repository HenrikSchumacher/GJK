#pragma once

#define CLASS CollisionFinder

namespace GJK
{
    
    template <int AMB_DIM, typename Real, typename Int, typename SReal>
    class CLASS
    {
        ASSERT_FLOAT(Real   );
        ASSERT_INT  (Int    );
        ASSERT_FLOAT(SReal  );
        
    public:
        
        using       Primitive_T =       PolytopeBase<AMB_DIM,GJK_Real,Int,SReal>;
        using MovingPrimitive_T = MovingPolytopeBase<AMB_DIM,GJK_Real,Int,SReal>;
        
        CLASS() {};

        CLASS(
            cref<MovingPrimitive_T> P_,
            cref<MovingPrimitive_T> Q_,
            const SReal TOL
        )
        :   P{ P_.Clone() }
        ,   Q{ Q_.Clone() }
        ,   eps(TOL)
        {}
        
        CLASS( const CLASS & other )
        :   P { other.P->Clone() }
        ,   Q { other.Q->Clone() }
        ,   eps(other.eps)
        {}
      
    protected:

        mutable std::shared_ptr<MovingPrimitive_T> P;
        mutable std::shared_ptr<MovingPrimitive_T> Q;
        
        mutable GJK_Algorithm<AMB_DIM+1,GJK_Real,Int> G;
        
        const SReal eps = static_cast<SReal>(0.0625);
        
        mutable Int max_iter = 128;
        mutable SReal b_stack[128] = {};

    public:
 
        SReal RelativeTolerance() const
        {
            return eps;
        }
        
//        void SetRelativeTolerance( const SReal eps_)
//        {
//            eps = eps_;
//        }
        
        SReal FindMaximumSafeStepSize(
            cptr<SReal> p, cptr<SReal> u,
            cptr<SReal> q, cptr<SReal> v,
            const SReal tinit,
            const bool reuse_direction = true
        ) const
        {
            GJK_tic(ClassName()+"::FindMaximumSafeStepSize");
            
            SReal a = Scalar::Zero<SReal>;
            SReal b = tinit;

            P->ReadCoordinatesSerialized(p);
            P->ReadVelocitiesSerialized(u);
            P->SetFirstTime(a);
            P->SetSecondTime(b);
            P->SetTimeScale(Scalar::One<SReal>);

            Q->ReadCoordinatesSerialized(q);
            Q->ReadVelocitiesSerialized(v);
            Q->SetFirstTime(a);
            Q->SetSecondTime(b);
            Q->SetTimeScale(Scalar::One<SReal>);

//            wprint(ClassName()+"::FindMaximumSafeStepSize: This might not work correctly for tinit !=1?!");
            
            SReal T = Sqrt(
                G.InteriorPoints_SquaredDistance(*P,*Q)
                + P->SquaredRadius()
                + Q->SquaredRadius()
//                - static_cast<Real>(0.75) * tinit * tinit
            )/tinit;

            P->SetTimeScale(T);
            Q->SetTimeScale(T);
            
            Int iter = 0;
            Int stack_ptr = 0;
            b_stack[0] = iter;
            

//            while( (b-a > eps * a) && (iter < max_iter) )
            while( (b-a > eps * b) && (iter < max_iter) )
            {
                ++iter;
                
                GJK_DUMP(iter);
                GJK_DUMP(a);
                GJK_DUMP(b);
                
                bool intersecting = G.IntersectingQ( *P, *Q, 1, reuse_direction && (iter>0) );
                
                GJK_DUMP(intersecting);
                
                if( !intersecting )
                {
                    if( stack_ptr > 0 )
                    {
                        a = b;
                        P->SetFirstTime(a);
                        Q->SetFirstTime(a);
                        
                        // pop
                        b = b_stack[stack_ptr--];
                        P->SetSecondTime(b);
                        Q->SetSecondTime(b);
                    }
                    else
                    {
                        GJK_print("Terminating because full step size is acceptable.");
                        GJK_toc(ClassName()+"::FindMaximumSafeStepSize");
                        return b;
                    }
                }
                else
                {
                    // push
                    b_stack[++stack_ptr] = b;
                    b = Scalar::Half<SReal> * (a + b);
                    
                    P->SetSecondTime(b);
                    Q->SetSecondTime(b);
                }
            }
            
            GJK_DUMP(iter);
            GJK_DUMP((b-a)/a);
            
            if( iter >= max_iter )
            {
//                wprint(ClassName()+"::FindMaximumSafeStepSize: max_iter = "+ToString(max_iter)+" reached." );
                GJK_toc(ClassName()+"::FindMaximumSafeStepSize");
                wprint(ClassName()+"::FindMaximumSafeStepSize: iter >= max_iter");
                return static_cast<SReal>(a);
            }
            
            if( a<= Scalar::Zero<SReal> )
            {
                GJK_print("Returning b.");
                GJK_toc(ClassName()+"::FindMaximumSafeStepSize");
                return static_cast<SReal>(b);
            }
            else
            {
                GJK_print("Returning a.");
                GJK_toc(ClassName()+"::FindMaximumSafeStepSize");
                return static_cast<SReal>(a);
            }
        }
        
    public:
        
        std::string ClassName() const
        {
            return TO_STD_STRING(CLASS)+"<"+ToString(AMB_DIM)+","+TypeName<Real>+","+TypeName<Int>+","+TypeName<SReal>+","+">";
        }
    };
    
    
} // GJK

#undef CLASS
