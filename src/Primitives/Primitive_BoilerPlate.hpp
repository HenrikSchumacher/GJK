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
//      begin = p_ + Size() * pos
// and
//      end   = p_ + Size() * (pos+1).
virtual void SetPointer( SReal * const p_, const Int pos ) override
{
    serialized_data = p_ + Size() * pos;
}

virtual void SetPointer( SReal * const p_ ) override
{
    serialized_data = p_;
}


virtual void Read( const SReal * const p_in, const Int i ) const override
{
    Read( p_in + Size() * i );
}

virtual void Read( const SReal * const p_in ) const override
{
    copy_buffer( p_in, serialized_data,  Size() );
}


virtual void Write(      SReal * const q_out, const Int j ) const override
{
    Write( q_out + Size() * j );
}

virtual void Write(      SReal * const q_out ) const override
{
    copy_buffer( serialized_data, q_out, Size() );
}


virtual void Swap( SReal * const p_out, const Int i, SReal * const q_out, const Int j ) const override
{
    Swap( p_out + Size() * i, q_out + Size() * j );
}

virtual void Swap( SReal * const p, SReal * const q ) const override
{
    std::swap_ranges( q, q + Size(), p );
}

virtual std::string DataString() const override
{
    std::stringstream s;
    
    const SReal * restrict const a = serialized_data;
    
    s << ClassName();
    s << ": data = { " << a[0];
    
    const Int k_begin = 1;
    const Int k_end   = Size();
    
    for( Int k = k_begin; k< k_end; ++k )
    {
        s << ", " << a[k];
    }
    s << " }";
    return s.str();
}
