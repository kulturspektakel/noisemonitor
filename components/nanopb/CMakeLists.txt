# Wraps upstream nanopb (vendored as a git submodule under upstream/) as an
# ESP-IDF component. We use upstream rather than the registry's
# livekit/nanopb because that fork bakes PB_BUFFER_ONLY=1 into its build,
# which forces nanopb to ignore stream callbacks and treat stream->state
# as a raw memory pointer — fatal when we want to stream-encode through
# a write() callback (e.g. record_writer.c writing to LittleFS).
idf_component_register(
    SRCS
        "upstream/pb_common.c"
        "upstream/pb_encode.c"
        "upstream/pb_decode.c"
    INCLUDE_DIRS "upstream"
)
