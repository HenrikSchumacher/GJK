//virtual CLASS * Clone() const override
//{
//    return new CLASS( *this );
//}

public:

using BASE::serialized_data;
//using BASE::Size;


public:

// Sets the classe's data pointer.
// We assume that p_ is an array of sufficient size in which the primitive's data is found between
//      begin = p_ + SIZE * pos
// and
//      end   = p_ + SIZE * (pos+1).
virtual void SetPointer( SReal * const p_, const Int pos ) override
{
    serialized_data = p_ + SIZE * pos;
}

virtual void SetPointer( SReal * const p_ ) override
{
    serialized_data = p_;
}


virtual void Read( const SReal * const p_in, const Int i ) const override
{
    Read( p_in + SIZE * i );
}

virtual void Read( const SReal * const p_in ) const override
{
    copy_buffer<SIZE>( p_in, serialized_data );
}


virtual void Write(      SReal * const q_out, const Int j ) const override
{
    Write( q_out + SIZE * j );
}

virtual void Write(      SReal * const q_out ) const override
{
    copy_buffer<SIZE>( serialized_data, q_out );
}


virtual void Swap( SReal * const p_out, const Int i, SReal * const q_out, const Int j ) const override
{
    Swap( p_out + SIZE * i, q_out + SIZE * j );
}

virtual void Swap( SReal * const p, SReal * const q ) const override
{
    std::swap_ranges( q, q + SIZE, p );
}

virtual std::string DataString() const override
{
    std::stringstream s;
    
    const SReal * restrict const a = serialized_data;
    
    s << ClassName();
    s << ": data = { " << a[0];
    
    for( Int k = 1; k < SIZE; ++k )
    {
        s << ", " << a[k];
    }
    s << " }";
    return s.str();
}
