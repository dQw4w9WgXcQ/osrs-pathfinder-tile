FROM rust as build
WORKDIR /workdir
COPY . .
RUN cargo build --package service --release

FROM ubuntu
WORKDIR /workdir
COPY --from=build /workdir/target/release/service /bindir/tile-service
EXPOSE 8080
CMD ["/bindir/tile-service"]