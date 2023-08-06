#pragma once

namespace GJK
{

    template<int AMB_DIM, typename Real, typename Int, typename SReal>
    void GJK_Offset_IntersectingQ_Batch
    (
        const Int n,                                   // number of primitive pairs
        const PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & P_, // prototype primitive
              SReal * const P_serialized_data,         // matrix of size n x (1 + AMB_DIM + POINT_COUNT_1 x AMB_DIM)
        const Real * const P_off_set,                  // vector of size n storing the thickness of the primitive
        const PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & Q_, // prototype primitive
              SReal * const Q_serialized_data,         // matrix of size n x (1 + AMB_DIM + POINT_COUNT_1 x AMB_DIM)
        const Real * const Q_off_set,                 // vector of size n storing the thickness of the primitive
              Int * const intersectingQ,               // vector of size n for storing the result
        const Int thread_count = 1
    )
    {
        tic("GJK_Offset_IntersectingQ_Batch");
        
        valprint("Number of primitive pairs",n);
        valprint("Ambient dimension        ",AMB_DIM);
        valprint("thread_count             ",thread_count);
        print("First  primitive type     = "+P_.ClassName());
        print("Second primitive type     = "+Q_.ClassName());

        const Int sub_calls = ParallelDoReduce(
            [&]( const Int thread ) -> Int
            {
                Int sub_calls (0);
                
                GJK_Algorithm<AMB_DIM,Real,Int> gjk;
                std::shared_ptr<PrimitiveSerialized<AMB_DIM,Real,Int,SReal>> P = P_.Clone();
                std::shared_ptr<PrimitiveSerialized<AMB_DIM,Real,Int,SReal>> Q = Q_.Clone();

                const Int i_begin = JobPointer<Int>( n, thread_count, thread    );
                const Int i_end   = JobPointer<Int>( n, thread_count, thread +1 );
                
                for( Int i = i_begin; i < i_end; ++i )
                {
                    //print(\"======================================================\");\ valprint(\" i \",i);

                    P->SetPointer( P_serialized_data, i );
                    Q->SetPointer( Q_serialized_data, i );

                    intersectingQ[i] = gjk.Offset_IntersectingQ( *P, P_off_set[i], *Q, Q_off_set[i] );
                    
                    sub_calls += gjk.SubCalls();
                }
                
                return sub_calls;
            },
            AddReducer<Int, Int>(),
            static_cast<Int>(0),
            thread_count
        );
        
        print("GJK_Offset_IntersectingQ_Batch made " + ToString(sub_calls) + " subcalls for n = " + ToString(n) + " primitive pairs.");
        toc("GJK_Offset_IntersectingQ_Batch");
    }
    
    template<int AMB_DIM,typename Real, typename Int, typename SReal>
    void GJK_Offset_SquaredDistances_Batch
    (
        const Int n,                                         // number of primitive pairs
        const PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & P_,       // prototype primitive
              SReal * const P_serialized_data,               // matrix of size n x (1 + AMB_DIM + POINT_COUNT_1 x AMB_DIM)
        const Real * const P_off_set,                        // vector of size n storing the thickness of the primitive
        const PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & Q_,       // prototype primitive
              SReal * const Q_serialized_data,               // matrix of size n x (1 + AMB_DIM + POINT_COUNT_1 x AMB_DIM)
        const Real * const Q_off_set,                        // vector of size n storing the thickness of the primitive
              Real * const squared_dist,                     // vector of size n for storing the squared distances
        const Int thread_count = 1
    )
    {
        tic("GJK_Offset_SquaredDistances_Batch");
        
        valprint("Number of primitive pairs",n);
        valprint("Ambient dimension        ",AMB_DIM);
        valprint("thread_count             ",thread_count);
        print("First  primitive type     = "+P_.ClassName());
        print("Second primitive type     = "+Q_.ClassName());

        const Int sub_calls = ParallelDoReduce(
            [&]( const Int thread ) -> Int
            {
                Int sub_calls (0);
                
                GJK_Algorithm<AMB_DIM,Real,Int> gjk;
                std::shared_ptr<PrimitiveSerialized<AMB_DIM,Real,Int,SReal>> P = P_.Clone();
                std::shared_ptr<PrimitiveSerialized<AMB_DIM,Real,Int,SReal>> Q = Q_.Clone();

                const Int i_begin = JobPointer<Int>( n, thread_count, thread    );
                const Int i_end   = JobPointer<Int>( n, thread_count, thread +1 );
                
                for( Int i = i_begin; i < i_end; ++i )
                {
                    //print(\"======================================================\");\ valprint(\" i \",i);

                    P->SetPointer( P_serialized_data, i );
                    Q->SetPointer( Q_serialized_data, i );

                    squared_dist[i] = gjk.Offset_SquaredDistance( *P, P_off_set[i], *Q, Q_off_set[i] );
                    
                    sub_calls += gjk.SubCalls();
                }
                
                return sub_calls;
            },
            AddReducer<Int, Int>(),
            static_cast<Int>(0),
            thread_count
        );

        print("GJK_Offset_SquaredDistances_Batch made " + ToString(sub_calls) + " subcalls for n = " + ToString(n) + " primitive pairs.");
        toc("GJK_Offset_SquaredDistances_Batch");
    }
    
    template<int AMB_DIM,typename Real, typename Int, typename SReal>
    void GJK_Offset_Witnesses_Batch
    (
        const Int n,                                         // number of primitive pairs
        const PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & P_,       // prototype primitive
              SReal * const P_serialized_data,               // matrix of size n x (1 + AMB_DIM + POINT_COUNT_1 x AMB_DIM)
        const Real * const P_off_set,                        // vector of size n storing the thickness of the primitive
              Real * const x,                                // matrix of size n x AMB_DIM for storing the witnesses in P
        const PrimitiveSerialized<AMB_DIM,Real,Int,SReal> & Q_,       // prototype primitive
              SReal * const Q_serialized_data,               // matrix of size n x (1 + AMB_DIM + POINT_COUNT_1 x AMB_DIM)
        const Real * const Q_off_set,                        // vector of size n storing the thickness of the primitive
              Real * const y,                                // matrix of size n x AMB_DIM for storing the witnesses in P
        const Int thread_count = 1
    )
    {
        tic("GJK_Offset_Witnesses_Batch");
        
        valprint("Number of primitive pairs",n);
        valprint("Ambient dimension        ",AMB_DIM);
        valprint("thread_count             ",thread_count);
        print(   "First  primitive type     = "+P_.ClassName());
        print(   "Second primitive type     = "+Q_.ClassName());

        const Int sub_calls = ParallelDoReduce(
            [&]( const Int thread ) -> Int
            {
                Int sub_calls (0);
                
                GJK_Algorithm<AMB_DIM,Real,Int> gjk;
                std::shared_ptr<PrimitiveSerialized<AMB_DIM,Real,Int,SReal>> P = P_.Clone();
                std::shared_ptr<PrimitiveSerialized<AMB_DIM,Real,Int,SReal>> Q = Q_.Clone();

                const Int i_begin = JobPointer<Int>( n, thread_count, thread    );
                const Int i_end   = JobPointer<Int>( n, thread_count, thread +1 );
                
                for( Int i = i_begin; i < i_end; ++i )
                {
                    //print(\"======================================================\");\ valprint(\" i \",i);

                    P->SetPointer( P_serialized_data, i );
                    Q->SetPointer( Q_serialized_data, i );

                    gjk.Offset_Witnesses( *P, P_off_set[i], x + AMB_DIM * i, *Q, Q_off_set[i], y + AMB_DIM * i );
                    
                    sub_calls += gjk.SubCalls();
                }

                
                return sub_calls;
            },
            AddReducer<Int, Int>(),
            static_cast<Int>(0),
            thread_count
        );

        print("GJK_Offset_Witnesses_Batch made " + ToString(sub_calls) + " subcalls for n = " + ToString(n) + " primitive pairs.");
        toc("GJK_Offset_Witnesses_Batch");
    }

} // namespe GJK
