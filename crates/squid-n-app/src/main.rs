fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "Squid-N",
        options,
        Box::new(|cc| {
            squid_n_app::app::install_japanese_fonts(&cc.egui_ctx);
            squid_n_app::theme::apply_theme(&cc.egui_ctx);
            Ok(Box::new(squid_n_app::app::App::default()))
        }),
    )
}
