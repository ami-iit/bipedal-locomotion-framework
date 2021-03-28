/**
 * @file Block.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_BLOCK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_BLOCK_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/impl/traits.h>

#include <atomic>
#include <memory>

namespace BipedalLocomotion
{
namespace System
{

/**
 * The description of empty signal
 */
using EmptyType = void*;

/**
 * Basic class that represents a block. A block is an abstraction of a generic block. It has a set
 * of input and a set of output. Please inherit block if you want to use it.
 * The custom block you design must implement. getOutput(), setInput(), advance() and close().
 * To avoid unnecessary overhead, Block is based on
 * [CRPT](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern) design patter.
 * To simplify the creation of your own block you should use the macro
 * #BLF_DEFINE_SYSTEM_BLOCK_INTERAL_STRUCTURE().
 * The following snippet can be used as an example to build a block.
 * \code{.cpp}
 * namespace BipedalLocomotion
 * {
 * namespace System
 * {
 * class DummyBlock;
 * }
 * }
 *
 * BLF_DEFINE_SYSTEM_BLOCK_INTERAL_STRUCTURE(DummyBlock, int, bool);
 *
 * namespace BipedalLocomotion
 * {
 * namespace System
 * {
 * class DummyBlock : public Block<DummyBlock>
 * {
 *     .....
 * };
 * } // namespace System
 * } // namespace BipedalLocomotion
 * \endcode
 */
template <class _Derived> class Block
{
    constexpr _Derived& derived()
    {
        return *static_cast<_Derived*>(this);
    }
    constexpr const _Derived& derived() const
    {
        return *static_cast<const _Derived*>(this);
    }

public:
    using Input = typename internal::traits<_Derived>::Input;
    using Output = typename internal::traits<_Derived>::Output;

    /**
     * Initialize the block.
     * @return true if the block has been initialed
     */
    virtual bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Get the output of the block
     * @note please define this function in your custom block
     * @return the output of the block
     */
    const Output& getOutput() const;

    /**
     * Set the input of the block.
     * @param input the input of the block
     * @note please define this function in your custom block
     * @return True in case of success and false otherwise.
     */
    bool setInput(const Input& input);

    /**
     * Advance the block.
     * @note please define this function in your custom block
     * @return True in case of success and false otherwise.
     */
    bool advance();

    /**
     * Close the block
     * @note please define this function in your custom block
     * @return True in case of success and false otherwise.
     */
    bool close();
};

template <class _Derived>
bool Block<_Derived>::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
}

template <class _Derived> const typename Block<_Derived>::Output& Block<_Derived>::getOutput() const
{
    return derived().getOutput();
}

template <class _Derived> bool Block<_Derived>::setInput(const Input& input)
{
    return derived().setInput(input);
}

template <class _Derived> bool Block<_Derived>::advance()
{
    return derived().advance();
}

template <class _Derived> bool Block<_Derived>::close()
{
    return derived().close();
}


} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_BLOCK_H
