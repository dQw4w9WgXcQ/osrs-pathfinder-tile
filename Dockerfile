#use docker buildx build NOT docker build

FROM rust as build
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "Building for $TARGETPLATFORM on $BUILDPLATFORM"
WORKDIR /workdir
COPY . .
RUN if [ -z "$TARGETPLATFORM" ]; then echo "TARGETPLATFORM not set"; exit 1; \
    elif [ "$TARGETPLATFORM" = "linux/amd64" ]; then CARGO_BUILD_TARGET="x86_64-unknown-linux-gnu";  \
    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then CARGO_BUILD_TARGET="aarch64-unknown-linux-gnu"; \
    else echo "Unsupported platform: $TARGETPLATFORM"; exit 1;  \
    fi; \
    echo "Building for $CARGO_BUILD_TARGET"; \
    cargo build --package service --release

FROM --platform=$TARGETPLATFORM ubuntu
WORKDIR /workdir
COPY --from=build /workdir/target/release/service /bindir/tile-service
EXPOSE 8080
CMD ["/bindir/tile-service"]