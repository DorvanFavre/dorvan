import 'package:dorvan/application/providers/language_state_provider.dart';
import 'package:dorvan/application/providers/translated_text_provider.dart';
import 'package:dorvan/domain/entity/language.dart';
import 'package:dorvan/presentation/app.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

void main() {
  runApp(ProviderScope(child: MyApp()));
}

/*
class HomePage extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Consumer(builder: (context, watch, child) {
                  final translatedText = watch(translatedTextProvider);
                  return Text(translatedText['title'] ?? '-');
                }),
                Consumer(builder: (context, watch, child) {
                  final translatedText = watch(translatedTextProvider);
                  return Text(translatedText['description'] ?? '-');
                })
              ],
            ),
          ),
          Align(
            alignment: Alignment.bottomLeft,
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                TextButton(
                    onPressed: () {
                      context.read(languageStateProvider).state = Language.EN;
                    },
                    child: Text('EN')),
                TextButton(
                    onPressed: () {
                      context.read(languageStateProvider).state = Language.FR;
                    },
                    child: Text('FR')),
                TextButton(
                    onPressed: () {
                      context.read(languageStateProvider).state = Language.DE;
                    },
                    child: Text('DE'))
              ],
            ),
          )
        ],
      ),
    );
  }
}
*/
