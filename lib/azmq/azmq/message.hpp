/*
    Copyright (c) 2013-2014 Contributors as noted in the AUTHORS file

    This file is part of azmq

    Distributed under the Boost Software License, Version 1.0. (See accompanying
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/
#ifndef AZMQ_MESSAGE_HPP__
#define AZMQ_MESSAGE_HPP__

#include "error.hpp"
#include "util/scope_guard.hpp"

#include <boost/assert.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/buffers_iterator.hpp>
#include <boost/system/system_error.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>

#include <zmq.h>

#include <type_traits>
#include <memory>
#include <vector>
#include <ostream>
#include <cstring>


namespace azmq {
namespace detail {
    struct socket_ops;
}

AZMQ_V1_INLINE_NAMESPACE_BEGIN

    struct nocopy_t {};

#ifdef BOOST_NO_CXX11_CONSTEXPR
    const nocopy_t nocopy = nocopy_t{};
#else
    constexpr nocopy_t nocopy = nocopy_t{};
#endif


    struct message {
        typedef void (free_fn) (void *data);

        using flags_type = int;

        message() BOOST_NOEXCEPT_OR_NOTHROW {
            auto rc = zmq_msg_init(&msg_);
            BOOST_ASSERT_MSG(rc == 0, "zmq_msg_init return non-zero"); (void)rc;
        }

        explicit message(size_t size) {
            auto rc = zmq_msg_init_size(&msg_, size);
            if (rc)
                throw boost::system::system_error(make_error_code());
        }

        message(boost::asio::const_buffer const& buffer) {
            auto sz = boost::asio::buffer_size(buffer);
            auto rc = zmq_msg_init_size(&msg_, sz);
            if (rc)
                throw boost::system::system_error(make_error_code());
            boost::asio::buffer_copy(boost::asio::buffer(zmq_msg_data(&msg_), sz),
                                     buffer);
        }

        message(nocopy_t, boost::asio::const_buffer const& buffer)
            : message(nocopy,
                boost::asio::mutable_buffer(
                    (void *)boost::asio::buffer_cast<const void*>(buffer),
                    boost::asio::buffer_size(buffer)),
                nullptr,
                nullptr)
        {}

        message(nocopy_t, boost::asio::mutable_buffer const& buffer, void* hint, zmq_free_fn* deleter)
        {
            auto rc = zmq_msg_init_data(&msg_,
                                        boost::asio::buffer_cast<void*>(buffer),
                                        boost::asio::buffer_size(buffer),
                                        deleter, hint);
            if (rc)
                throw boost::system::system_error(make_error_code());
        }

        template<class Deleter, class Enabler =
            typename std::enable_if<!std::is_convertible<Deleter, free_fn *>::value>::type
        >
        message(nocopy_t, boost::asio::mutable_buffer const& buffer, Deleter&& deleter)
        {
            using D = typename std::decay<Deleter>::type;

            const auto call_deleter = [](void *buf, void *hint) {
                std::unique_ptr<D> deleter(reinterpret_cast<D*>(hint));
                BOOST_ASSERT_MSG(deleter, "!deleter");
                (*deleter)(buf);
            };

            std::unique_ptr<D> d(new D(std::forward<Deleter>(deleter)));
            auto rc = zmq_msg_init_data(&msg_,
                                        boost::asio::buffer_cast<void*>(buffer),
                                        boost::asio::buffer_size(buffer),
                                        call_deleter, d.get());
            if (rc)
                throw boost::system::system_error(make_error_code());
            d.release();
        }

        message(nocopy_t, boost::asio::mutable_buffer const& buffer, free_fn* deleter)
        {
            BOOST_ASSERT_MSG(deleter, "!deleter");

            const auto call_deleter = [](void *buf, void *hint) {
                free_fn *deleter(reinterpret_cast<free_fn*>(hint));
                (*deleter)(buf);
            };

            auto rc = zmq_msg_init_data(&msg_,
                                        boost::asio::buffer_cast<void*>(buffer),
                                        boost::asio::buffer_size(buffer),
                                        call_deleter, reinterpret_cast<void *>(deleter));
            if (rc)
                throw boost::system::system_error(make_error_code());
        }

        explicit message(std::string const& str)
            : message(boost::asio::buffer(str))
        { }

        message(message && rhs) BOOST_NOEXCEPT_OR_NOTHROW
            : msg_(rhs.msg_)
        {
            auto rc = zmq_msg_init(&rhs.msg_);
            BOOST_ASSERT_MSG(rc == 0, "zmq_msg_init return non-zero"); (void)rc;
        }

        message& operator=(message && rhs) BOOST_NOEXCEPT_OR_NOTHROW {
            msg_ = rhs.msg_;
            auto rc = zmq_msg_init(&rhs.msg_);
            BOOST_ASSERT_MSG(rc == 0, "zmq_msg_init return non-zero"); (void)rc;

            return *this;
        }

        message(message const& rhs) {
            auto rc = zmq_msg_init(const_cast<zmq_msg_t*>(&msg_));
            BOOST_ASSERT_MSG(rc == 0, "zmq_msg_init return non-zero");
            rc = zmq_msg_copy(const_cast<zmq_msg_t*>(&msg_),
                              const_cast<zmq_msg_t*>(&rhs.msg_));
            if (rc)
                throw boost::system::system_error(make_error_code());
        }

        message& operator=(message const& rhs) {
            auto rc = zmq_msg_copy(const_cast<zmq_msg_t*>(&msg_),
                                   const_cast<zmq_msg_t*>(&rhs.msg_));
            if (rc)
                throw boost::system::system_error(make_error_code());
            return *this;
        }

        ~message() { close(); }

        boost::asio::const_buffer cbuffer() const {
            auto pv = zmq_msg_data(const_cast<zmq_msg_t*>(&msg_));
            return boost::asio::buffer(pv, size());
        }

        operator boost::asio::const_buffer() const {
            return cbuffer();
        }

        boost::asio::const_buffer buffer() const {
            auto pv = zmq_msg_data(const_cast<zmq_msg_t*>(&msg_));
            return boost::asio::buffer(pv, size());
        }

        boost::asio::mutable_buffer buffer() {
            if (is_shared())
                deep_copy();

            auto pv = zmq_msg_data(const_cast<zmq_msg_t*>(&msg_));
            return boost::asio::buffer(pv, size());
        }

        template<typename T>
        T const& buffer_cast() const {
            return *boost::asio::buffer_cast<T const*>(buffer());
        }

        size_t buffer_copy(boost::asio::mutable_buffer const& target) const {
            return boost::asio::buffer_copy(target, buffer());
        }

        bool operator==(const message & rhs) const {
            auto pa = reinterpret_cast<char*>(
                                zmq_msg_data(const_cast<zmq_msg_t*>(&msg_)));
            auto pb = reinterpret_cast<char*>(
                                zmq_msg_data(const_cast<zmq_msg_t*>(&rhs.msg_)));
            return std::equal(pa, pa + size(), pb);
        }

        std::string string() const {
            auto pv = zmq_msg_data(const_cast<zmq_msg_t*>(&msg_));
            auto sz = size();
            return std::string(reinterpret_cast<char const*>(pv), sz);
        }

        friend
        std::ostream& operator<<(std::ostream& stm, message const& that) {
            return stm << "message{sz=" << that.size() << "}";
        }

        size_t size() const { return zmq_msg_size(const_cast<zmq_msg_t*>(&msg_)); }

        void rebuild() {
            close();
            auto rc = zmq_msg_init(&msg_);
            if (rc)
                throw boost::system::system_error(make_error_code());
        }

        void rebuild(size_t size) {
            close();
            auto rc = zmq_msg_init_size(&msg_, size);
            if (rc)
                throw boost::system::system_error(make_error_code());
        }

        void rebuild(boost::asio::const_buffer const& buffer) {
            close();
            auto sz = boost::asio::buffer_size(buffer);
            auto rc = zmq_msg_init_size(&msg_, sz);
            if (rc)
                throw boost::system::system_error(make_error_code());
            boost::asio::buffer_copy(boost::asio::buffer(zmq_msg_data(&msg_), sz),
                                     buffer);
        }

        void rebuild(std::string const& str) { rebuild(boost::asio::buffer(str)); }

        bool more() const {
            return zmq_msg_more(const_cast<zmq_msg_t*>(&msg_)) ? true : false;
        }

    private:
        friend detail::socket_ops;
        zmq_msg_t msg_;

        /*template<class Deleter>
        static void call_deleter(void *pv, void* hint) {
            std::unique_ptr<Deleter> deleter(reinterpret_cast<Deleter*>(hint));
            BOOST_ASSERT_MSG(deleter, "!deleter");
            (*deleter)(pv);
        }*/

        void close() {
            auto rc = zmq_msg_close(&msg_);
            if (rc)
                throw boost::system::system_error(make_error_code());
        }

        // note, this is a bit fragile, currently the last two bytes in a
        // zmq_msg_t hold the type and flags fields
        enum {
            flags_offset = sizeof(zmq_msg_t) - 1,
            type_offset = sizeof(zmq_msg_t) - 2
        };

        uint8_t flags() const {
            auto pm = const_cast<zmq_msg_t*>(&msg_);
            auto p = reinterpret_cast<uint8_t*>(pm);
            return p[flags_offset];
        }

        uint8_t type() const {
            auto pm = const_cast<zmq_msg_t*>(&msg_);
            auto p = reinterpret_cast<uint8_t*>(pm);
            return p[type_offset];
        }

        // flags and type we care about from msg.hpp in zeromq lib
        enum
        {
            flag_shared = 128,
            type_cmsg = 104
        };

        bool is_shared() const {
            // TODO use shared message property if libzmq version >= 4.1
            return (flags() & flag_shared) || type() == type_cmsg;
        }

        void deep_copy() {
            auto sz = size();
            zmq_msg_t tmp;
            auto rc = zmq_msg_init(&tmp);
            BOOST_ASSERT_MSG(rc == 0, "zmq_msg_init return non-zero");

            rc = zmq_msg_move(&tmp, &msg_);
            BOOST_ASSERT_MSG(rc == 0, "zmq_msg_move return non-zero");

            // ensure that tmp is always cleaned up
            SCOPE_EXIT { zmq_msg_close(&tmp); };

            rc = zmq_msg_init_size(&msg_, sz);
            if (rc)
                throw boost::system::system_error(make_error_code());

            auto pdst = zmq_msg_data(const_cast<zmq_msg_t*>(&msg_));
            auto psrc = zmq_msg_data(&tmp);
            ::memcpy(pdst, psrc, sz);
        }
    };

    template<typename IteratorType>
    struct const_message_iterator
        : boost::iterator_facade<
                    const_message_iterator<IteratorType>
                , message const
                , boost::forward_traversal_tag
            >
    {
        const_message_iterator(IteratorType const& it)
            : it_(it)
        { }

    private:
        friend class boost::iterator_core_access;

        IteratorType it_;
        mutable message msg_;

        void increment() { it_++; }
        bool equal(const_message_iterator const& other) const {
            return it_ == other.it_;
        }

        message& dereference() const {
            msg_.rebuild(*it_);
            return msg_;
        }
    };

    template<typename BufferSequence>
    struct deduced_const_message_range {
        using iterator_type = const_message_iterator<typename BufferSequence::const_iterator>;
        using range_type = boost::iterator_range<iterator_type>;

        static range_type get_range(BufferSequence const& buffers) {
            return range_type(iterator_type(std::begin(buffers)),
                              iterator_type(std::end(buffers)));
        }
    };

    // note, as with asio's expectations, buffers must remain valid for the lifetime of the returned
    // range
    template<typename ConstBufferSequence>
    static auto const_message_range(ConstBufferSequence const& buffers) ->
        typename deduced_const_message_range<ConstBufferSequence>::range_type
    {
        return deduced_const_message_range<ConstBufferSequence>::get_range(buffers);
    }

    using message_vector = std::vector<message>;

    template<typename BufferSequence>
    message_vector to_message_vector(BufferSequence const& buffers) {
        return message_vector(std::begin(buffers), std::end(buffers));
    }
AZMQ_V1_INLINE_NAMESPACE_END
} // namespace azmq
#endif // AZMQ_MESSAGE_HPP__
